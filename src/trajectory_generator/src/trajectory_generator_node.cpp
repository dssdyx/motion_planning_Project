#include <algorithm>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <random>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Useful customized headers
#include "Astar_searcher.h"
#include "backward.hpp"
#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

TrajectoryGeneratorWaypoint *_trajGene = new TrajectoryGeneratorWaypoint();
AstarPathFinder *_astar_path_finder = new AstarPathFinder();

// Set the obstacle map
double _resolution, _inv_resolution, _path_resolution;
double _x_size, _y_size, _z_size;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// Param from launch file
double _vis_traj_width;
double _Vel, _Acc;
int _dev_order, _min_order;

// ros related
ros::Subscriber _map_sub, _pts_sub, _odom_sub;
ros::Publisher _traj_vis_pub,_traj_vis_pub_r, _traj_pub, _path_vis_pub,_path_vis_pub_r;

// for planning
Vector3d odom_pt, odom_vel, start_pt, target_pt, start_vel;
double height;
int _poly_num1D;
MatrixXd _polyCoeff;
VectorXd _polyTime;
double time_duration;
ros::Time time_traj_start;
bool has_odom = false;
bool has_target = false;

// for replanning
enum STATE {
  INIT,
  WAIT_TARGET,
  GEN_NEW_TRAJ,
  EXEC_TRAJ,
  REPLAN_TRAJ
} exec_state = STATE::INIT;
double no_replan_thresh, replan_thresh;
ros::Timer _exec_timer;
void execCallback(const ros::TimerEvent &e);

// declare
void changeState(STATE new_state, string pos_call);
void printState();
void visTrajectory(MatrixXd polyCoeff, VectorXd time);
void visTrajectory_r(MatrixXd polyCoeff, VectorXd time);
void visPath(MatrixXd nodes);
void visPath_r(MatrixXd nodes);
void trajOptimization(Eigen::MatrixXd path);
void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &odom);
void rcvWaypointsCallback(const nav_msgs::Path &wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);
void trajPublish(MatrixXd polyCoeff, VectorXd time);
bool trajGeneration();
VectorXd timeAllocation(MatrixXd Path);
Vector3d getPos(double t_cur);
Vector3d getVel(double t_cur);

// change the state to the new state
void changeState(STATE new_state, string pos_call) {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  int pre_s = int(exec_state);
  exec_state = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " +
              state_str[int(new_state)]
       << endl;
}

void printState() {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  cout << "[Clock]: state: " + state_str[int(exec_state)] << endl;
}

void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  odom_pt(0) = odom->pose.pose.position.x;
  odom_pt(1) = odom->pose.pose.position.y;
  odom_pt(2) = odom->pose.pose.position.z;

  odom_vel(0) = odom->twist.twist.linear.x;
  odom_vel(1) = odom->twist.twist.linear.y;
  odom_vel(2) = odom->twist.twist.linear.z;

  has_odom = true;
}

// Control the State changes
void execCallback(const ros::TimerEvent &e) {
  static int num = 0;
  num++;
  if (num == 100) {
    printState();
    if (!has_odom)
      cout << "no odom." << endl;
    if (!has_target)
      cout << "wait for goal." << endl;
    num = 0;
  }

  switch (exec_state) {
  case INIT: {
    if (!has_odom)
      return;
    if (!has_target)
      return;
    changeState(WAIT_TARGET, "STATE");
    break;
  }

  case WAIT_TARGET: {
    if (!has_target)
      return;
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }

  case GEN_NEW_TRAJ: {
    bool success = trajGeneration();
    if (success)
      changeState(EXEC_TRAJ, "STATE");
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }

  case EXEC_TRAJ: {
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - time_traj_start).toSec();
    double t_replan = ros::Duration(1, 0).toSec();
    t_cur = min(time_duration, t_cur);

    if (t_cur > time_duration - 1e-2) {
      has_target = false;
      changeState(WAIT_TARGET, "STATE");
      return;
    } else if ((target_pt - odom_pt).norm() < no_replan_thresh) {
      return;
    } else if ((start_pt - odom_pt).norm() < replan_thresh) {
      return;
    } else if (t_cur < t_replan) {
      return;
    } else {
      changeState(REPLAN_TRAJ, "STATE");
    }
    break;
  }
  case REPLAN_TRAJ: {
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - time_traj_start).toSec();
    double t_delta = ros::Duration(0, 50).toSec();
    t_cur = t_delta + t_cur;
    //start_pt = getPos(t_cur);
    //start_vel = getVel(t_cur);
    start_pt = odom_pt;
    start_pt(2) = height;
    start_vel = odom_vel;
    start_vel(2) = 0;
    //cout<<"start_pt: "<<endl<<start_pt<<endl;
    bool success = trajGeneration();
    if (success)
      changeState(EXEC_TRAJ, "STATE");
    else
      changeState(GEN_NEW_TRAJ, "STATE");
    break;
  }
  }
}

void rcvWaypointsCallBack(const nav_msgs::Path &wp) {
  if (wp.poses[0].pose.position.z < 0.0)
    return;
  target_pt << wp.poses[0].pose.position.x, wp.poses[0].pose.position.y,
      height;//wp.poses[0].pose.position.z;
  ROS_INFO("[node] receive the planning target");
  start_pt = odom_pt;
  start_pt(2) = height;
  start_vel = odom_vel;
  start_vel(2) = 0;
  has_target = true;

  if (exec_state == WAIT_TARGET)
    changeState(GEN_NEW_TRAJ, "STATE");
  else if (exec_state == EXEC_TRAJ)
    changeState(REPLAN_TRAJ, "STATE");
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;
  sensor_msgs::PointCloud2 map_vis;

  pcl::fromROSMsg(pointcloud_map, cloud);

  if ((int)cloud.points.size() == 0)
    return;

  pcl::PointXYZ pt;
  for (int idx = 0; idx < (int)cloud.points.size(); idx++) {
    pt = cloud.points[idx];
    // set obstalces into grid map for path planning
    _astar_path_finder->setObs(pt.x, pt.y, pt.z);
  }
}

// trajectory generation: front-end + back-end
// front-end : A* search method
// back-end  : Minimum snap trajectory generation
bool trajGeneration() {
  /**
   *
   * STEP 1:  search the path and get the path
   *
   * **/
   //cout<<"start: "<<endl<<start_pt<<endl;
   //cout<<"target:"<<endl<<target_pt<<endl;
    ROS_INFO("Traj_generation");
  _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
  //_astar_path_finder->resetUsedGrids();
  auto grid_path = _astar_path_finder->getPath();
  cout<<"a path size: "<<grid_path.size()<<endl;
  if(grid_path.empty())    return -1;
  auto visited_nodes = _astar_path_finder->getVisitedNodes();


  // Reset map for next call

  /**
   *
   * STEP 2:  Simplify the path: use the RDP algorithm
   *
   * **/
  ROS_INFO("now begin to simplify the path!");
  cout<<"before simply: "<<grid_path.size()<<endl;
  MatrixXd Apath(int(grid_path.size()), 3);
  for (int k = 0; k < int(grid_path.size()); k++) {
    Apath.row(k) = grid_path[k];
  }
  //visPath(Apath);
  grid_path = _astar_path_finder->pathSimplify(grid_path, _path_resolution);
  cout<<"after simply: "<<grid_path.size()<<endl;
  ROS_INFO("simplify the path finished");
  MatrixXd path(int(grid_path.size()), 3);
  for (int k = 0; k < int(grid_path.size()); k++) {
    path.row(k) = grid_path[k];
  }

  /**
   *
   * STEP 3:  Trajectory optimization
   *
   * **/
  trajOptimization(path);
  time_duration = _polyTime.sum();

  // Publish the trajectory
  trajPublish(_polyCoeff, _polyTime);
  // record the trajectory start time
  time_traj_start = ros::Time::now();
  // return if the trajectory generation successes
  if (_polyCoeff.rows() > 0)
    return true;
  else
    return false;
}

void trajOptimization(Eigen::MatrixXd path) {
  // if( !has_odom ) return;
  MatrixXd A_path = path;
  MatrixXd vel = MatrixXd::Zero(2, 3);
  MatrixXd acc = MatrixXd::Zero(2, 3);

  vel.row(0) = start_vel;

  /**
   *
   * STEP 3.1:  finish the timeAllocation() using resonable allocation
   *
   * **/
  //ROS_INFO("begin time allocation");
  _polyTime = timeAllocation(path);
  //ROS_INFO("time allocation finised");

  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
  //ROS_INFO("begin polycoffe get");
  _polyCoeff =
      _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);
  //ROS_INFO("end polycoffe get");
  // check if the trajectory is safe, if not, do reoptimize
  int unsafe_segment;

  /**
   *
   * STEP 3.3:  finish the safeCheck()
   *
   * **/

  //ROS_INFO("begin safe check");
  unsafe_segment = _astar_path_finder->safeCheck(_polyCoeff, _polyTime);
  cout<<"unsafe segment :"<<unsafe_segment<<endl;

  MatrixXd repath = path;
  int count = 0;
  cout<<"path size: "<<path.rows()<<endl;
  VectorXd reopt_polyTime = _polyTime;
  MatrixXd reopt_polyCoeff = _polyCoeff;
  MatrixXd path_reopt = path;
  int count_reopt = 0;
  double path_resolution_iter = _path_resolution;
  //use A* reopt

  while(unsafe_segment != -1)
  {
      count_reopt ++;
      path_resolution_iter /=2;
      int j = unsafe_segment;
      Vector3d unsafe_start_pt = _astar_path_finder->getPosPoly(reopt_polyCoeff,j,0);
      Vector3d unsafe_end_pt = _astar_path_finder->getPosPoly(reopt_polyCoeff,j,reopt_polyTime(j));
      _astar_path_finder->AstarGraphSearch(unsafe_start_pt, unsafe_end_pt);
      auto reopt_grid_path = _astar_path_finder->getPath();
      cout<<"new insert point num without simply: "<<reopt_grid_path.size()-2<<endl;
      cout<<"path_resolution: "<<path_resolution_iter<<endl;
      reopt_grid_path = _astar_path_finder->pathSimplify(reopt_grid_path, path_resolution_iter);
      int new_wp_num = reopt_grid_path.size() -2;
      cout<<"new insert point num: "<<new_wp_num<<endl;
      MatrixXd path_tmp;
      if(new_wp_num !=0)
      {
          path_tmp = MatrixXd::Zero(path_reopt.rows()+ new_wp_num ,3);
          path_tmp.topRows(j+1) = path_reopt.topRows(j+1);
          for(int i = j+1; i<= j+1 + new_wp_num -1;i++)
          {
            path_tmp.row(i) = reopt_grid_path[i - j];
          }
          path_tmp.bottomRows(path_reopt.rows()-j-1)=path_reopt.bottomRows(path_reopt.rows()-j-1);
      }
      else
      {
          path_tmp = MatrixXd::Zero(path_reopt.rows()+ 1 ,3);
          path_tmp.topRows(j+1) = path_reopt.topRows(j+1);
          path_tmp.row(j+1) = path_reopt.row(j)/2+path_reopt.row(j+1)/2;
          path_tmp.bottomRows(path_reopt.rows()-j-1)=path_reopt.bottomRows(path_reopt.rows()-j-1);
      }


      reopt_polyTime = timeAllocation(path_tmp);
      reopt_polyCoeff =
          _trajGene->PolyQPGeneration(_dev_order, path_tmp, vel, acc, reopt_polyTime);
      cout<<"unsafe start idx: "<<endl<<_astar_path_finder->coord2gridIndex(unsafe_start_pt)<<endl;
      unsafe_segment = _astar_path_finder->safeCheck(reopt_polyCoeff, reopt_polyTime);
      cout<<"unsafe end idx: "<<endl<<_astar_path_finder->coord2gridIndex(unsafe_end_pt)<<endl;
      //unsafe_segment = -1;
      path_reopt = path_tmp;
      visPath_r(path_reopt);
      visTrajectory_r(reopt_polyCoeff,reopt_polyTime);
      //if(unsafe_segment !=-1)

      cout<<"count_reopt: "<<count_reopt<<endl;
      cout<<"unsafe_segment: "<<unsafe_segment<<endl;
      cout<<"path_reopt size: "<<path_reopt.rows()<<endl;
      if(count_reopt >50 || unsafe_segment == -1)
      {
          _polyCoeff = reopt_polyCoeff;
          _polyTime = reopt_polyTime;
          path = path_reopt;
          break;
      }
  }


  /*
  while (unsafe_segment != -1) {
      cout<<"*********"<<endl;
      count_reopt ++;
      int j = unsafe_segment;
      MatrixXd path_tmp = MatrixXd::Zero(path_reopt.rows()+1,3);
      //path_tmp.topRows(j+1) = path_reopt.topRows(j+1);
      //path_tmp.row(j+1) << path_reopt.row(j)/2+path_reopt.row(j+1)/2;
      //path_tmp.bottomRows(path_reopt.rows()-j-1)=path_reopt.bottomRows(path_reopt.rows()-j-1);
      //cout<<"path: "<<endl<<path<<endl;
      //cout<<"path_opt: "<<endl<<path_reopt<<endl;
      for(int i = 0;i<=j;i++)
      {
          path_tmp.row(i) = path_reopt.row(i);
      }
      path_tmp.row(j+1) = path_reopt.row(j)/2+path_reopt.row(j+1)/2;
      for(int i = j+2;i<path_tmp.rows();i++)
      {
          path_tmp.row(i) = path_reopt.row(i-1);
      }
      cout<<"path size: "<<path_reopt.rows()<<endl;
      cout<<"seg : "<<reopt_polyCoeff.rows()<<endl;
      reopt_polyTime = timeAllocation(path_tmp);
      reopt_polyCoeff =
          _trajGene->PolyQPGeneration(_dev_order, path_tmp, vel, acc, reopt_polyTime);
      Vector3d unsafe_start_pt = path_reopt.row(j);
      Vector3d unsafe_end_pt = path_reopt.row(j+1);
      Vector3i unsafe_start_idx = _astar_path_finder->coord2gridIndex(unsafe_start_pt);
      Vector3i unsafe_end_idx = _astar_path_finder->coord2gridIndex(unsafe_end_pt);
      cout<<"unsafe start idx: "<<endl<<unsafe_start_idx<<endl;
      unsafe_segment = _astar_path_finder->safeCheck(reopt_polyCoeff, reopt_polyTime);
      cout<<"unsafe end idx: "<<endl<<unsafe_end_idx<<endl;
      //unsafe_segment = -1;
      path_reopt = path_tmp;
      visPath_r(path_reopt);
      visTrajectory_r(reopt_polyCoeff,reopt_polyTime);
      //if(unsafe_segment !=-1)

      cout<<"count_reopt: "<<count_reopt<<endl;
      cout<<"unsafe_segment: "<<unsafe_segment<<endl;
      cout<<"path_reopt size: "<<path_reopt.rows()<<endl;
      if(count_reopt >50 || unsafe_segment == -1) break;
  }
  */





  //ROS_INFO("end safe check");
  // visulize path and trajectory
  //ROS_INFO("show repath");
  visPath(path);
  //ROS_INFO("end show path");
  visTrajectory(_polyCoeff, _polyTime);
  //ROS_INFO("end show poly");
}

void trajPublish(MatrixXd polyCoeff, VectorXd time) {
  if (polyCoeff.size() == 0 || time.size() == 0) {
    ROS_WARN("[trajectory_generator_waypoint] empty trajectory, nothing to "
             "publish.");
    return;
  }

  unsigned int poly_number;

  static int count =
      1; // The first trajectory_id must be greater than 0. zxzxzxzx

  quadrotor_msgs::PolynomialTrajectory traj_msg;

  traj_msg.header.seq = count;
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.header.frame_id = std::string("/world");
  traj_msg.trajectory_id = count;
  traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

  traj_msg.num_order = 2 * _dev_order - 1; // the order of polynomial
  traj_msg.num_segment = time.size();

  Vector3d initialVel, finalVel;
  initialVel = _trajGene->getVelPoly(_polyCoeff, 0, 0);
  finalVel = _trajGene->getVelPoly(_polyCoeff, traj_msg.num_segment - 1,
                                   _polyTime(traj_msg.num_segment - 1));
  traj_msg.start_yaw = atan2(initialVel(1), initialVel(0));
  traj_msg.final_yaw = atan2(finalVel(1), finalVel(0));

  poly_number = traj_msg.num_order + 1;
  // cout << "p_order:" << poly_number << endl;
  // cout << "traj_msg.num_order:" << traj_msg.num_order << endl;
  // cout << "traj_msg.num_segment:" << traj_msg.num_segment << endl;
  for (unsigned int i = 0; i < traj_msg.num_segment; i++) {
    for (unsigned int j = 0; j < poly_number; j++) {
      traj_msg.coef_x.push_back(polyCoeff(i, j) * pow(time(i), j));
      traj_msg.coef_y.push_back(polyCoeff(i, poly_number + j) *
                                pow(time(i), j));
      traj_msg.coef_z.push_back(polyCoeff(i, 2 * poly_number + j) *
                                pow(time(i), j));
    }
    traj_msg.time.push_back(time(i));
    traj_msg.order.push_back(traj_msg.num_order);
  }
  traj_msg.mag_coeff = 1;

  count++;
  ROS_WARN("[traj..gen...node] traj_msg publish");
  _traj_pub.publish(traj_msg);
}

VectorXd timeAllocation(MatrixXd Path) {
    /*
  VectorXd time(Path.rows() - 1);
  for(size_t i=0;i<time.size();i++)
  {
      time(i) = (Path.row(i+1) - Path.row(i)).norm()/_Vel;
  }
  time(0) *= 2.0;
  time(0) = max(1.0,time(0));
  time(time.rows()-1) *= 2.0;
  time(time.rows()-1) = max(1.0,time(time.rows()-1));

  return time;
  */
    VectorXd time(Path.rows() - 1);
    double t_slope = _Vel / _Acc;
    double max_s_triangle =  t_slope * _Vel;
    //std::cout <<"Path: "<< Path << std::endl;

    time = VectorXd::Ones(Path.rows() - 1);
    for(int k = 0;k < Path.rows()-1;k++){
        Vector3d p1_2 = Path.row(k) - Path.row(k+1);
        double s = std::sqrt(p1_2.dot(p1_2));

        if(s <= max_s_triangle){
            time(k) = 2.0 * s / _Acc;
        }
        else{
            time(k) = (s - max_s_triangle) / _Vel + t_slope;
        }
    }
    //std::cout <<"time: "<< time << std::endl;
    return time;
}

void visTrajectory(MatrixXd polyCoeff, VectorXd time) {
  visualization_msgs::Marker _traj_vis;

  _traj_vis.header.stamp = ros::Time::now();
  _traj_vis.header.frame_id = "/world";

  _traj_vis.ns = "traj_node/trajectory";
  _traj_vis.id = 0;
  _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
  _traj_vis.action = visualization_msgs::Marker::ADD;
  _traj_vis.scale.x = _vis_traj_width;
  _traj_vis.scale.y = _vis_traj_width;
  _traj_vis.scale.z = _vis_traj_width;
  _traj_vis.pose.orientation.x = 0.0;
  _traj_vis.pose.orientation.y = 0.0;
  _traj_vis.pose.orientation.z = 0.0;
  _traj_vis.pose.orientation.w = 1.0;

  _traj_vis.color.a = 1.0;
  _traj_vis.color.r = 0.0;
  _traj_vis.color.g = 0.5;
  _traj_vis.color.b = 1.0;

  _traj_vis.points.clear();
  Vector3d pos;
  geometry_msgs::Point pt;

  for (int i = 0; i < time.size(); i++) {
    for (double t = 0.0; t < time(i); t += 0.01) {
      pos = _trajGene->getPosPoly(polyCoeff, i, t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      _traj_vis.points.push_back(pt);
    }
  }
  _traj_vis_pub.publish(_traj_vis);
}
void visTrajectory_r(MatrixXd polyCoeff, VectorXd time) {
  visualization_msgs::Marker _traj_vis;

  _traj_vis.header.stamp = ros::Time::now();
  _traj_vis.header.frame_id = "/world";

  _traj_vis.ns = "traj_node/trajectory";
  _traj_vis.id = 0;
  _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
  _traj_vis.action = visualization_msgs::Marker::ADD;
  _traj_vis.scale.x = _vis_traj_width;
  _traj_vis.scale.y = _vis_traj_width;
  _traj_vis.scale.z = _vis_traj_width;
  _traj_vis.pose.orientation.x = 0.0;
  _traj_vis.pose.orientation.y = 0.0;
  _traj_vis.pose.orientation.z = 0.0;
  _traj_vis.pose.orientation.w = 1.0;

  _traj_vis.color.a = 1.0;
  _traj_vis.color.r = 1.0;
  _traj_vis.color.g = 0.0;
  _traj_vis.color.b = 0.0;

  _traj_vis.points.clear();
  Vector3d pos;
  geometry_msgs::Point pt;

  for (int i = 0; i < time.size(); i++) {
    for (double t = 0.0; t < time(i); t += 0.01) {
      pos = _trajGene->getPosPoly(polyCoeff, i, t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      _traj_vis.points.push_back(pt);
    }
  }
  _traj_vis_pub_r.publish(_traj_vis);
}

void visPath_r(MatrixXd nodes) {
  visualization_msgs::Marker points;

  int id = 0;
  points.id = id;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.header.frame_id = "world";
  points.header.stamp = ros::Time::now();
  points.ns = "traj_node/path";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.scale.z = 0.2;
  points.color.a = 1.0;
  points.color.r = 1.0;
  points.color.g = 0.0;
  points.color.b = 0.0;

  geometry_msgs::Point p;
  for (int i = 0; i < int(nodes.rows()); i++) {
    p.x = nodes(i, 0);
    p.y = nodes(i, 1);
    p.z = nodes(i, 2);

    points.points.push_back(p);
  }
  _path_vis_pub_r.publish(points);
}
void visPath(MatrixXd nodes) {
  visualization_msgs::Marker points;

  int id = 0;
  points.id = id;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.header.frame_id = "world";
  points.header.stamp = ros::Time::now();
  points.ns = "traj_node/path";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.scale.z = 0.2;
  points.color.a = 1.0;
  points.color.r = 0.0;
  points.color.g = 0.0;
  points.color.b = 1.0;

  geometry_msgs::Point p;
  for (int i = 0; i < int(nodes.rows()); i++) {
    p.x = nodes(i, 0);
    p.y = nodes(i, 1);
    p.z = nodes(i, 2);

    points.points.push_back(p);
  }
  _path_vis_pub.publish(points);
}

Vector3d getPos(double t_cur) {
  double time = 0;
  Vector3d pos = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        pos = _trajGene->getPosPoly(_polyCoeff, i, t);
        return pos;
      }
    }
  }
  return pos;
}

Vector3d getVel(double t_cur) {
  double time = 0;
  Vector3d Vel = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        Vel = _trajGene->getVelPoly(_polyCoeff, i, t);
        return Vel;
      }
    }
  }
  return Vel;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_node");
  ros::NodeHandle nh("~");

  nh.param("planning/height", height, 1.0);
  nh.param("planning/vel", _Vel, 1.0);
  nh.param("planning/acc", _Acc, 1.0);
  nh.param("planning/dev_order", _dev_order, 3);
  nh.param("planning/min_order", _min_order, 3);
  nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
  nh.param("map/resolution", _resolution, 0.2);
  nh.param("map/x_size", _x_size, 50.0);
  nh.param("map/y_size", _y_size, 50.0);
  nh.param("map/z_size", _z_size, 5.0);
  nh.param("path/resolution", _path_resolution, 0.05);
  nh.param("replanning/thresh_replan", replan_thresh, -1.0);
  nh.param("replanning/thresh_no_replan", no_replan_thresh, -1.0);

  _poly_num1D = 2 * _dev_order;

  _exec_timer = nh.createTimer(ros::Duration(0.01), execCallback);

  _odom_sub = nh.subscribe("odom", 10, rcvOdomCallback);
  _map_sub = nh.subscribe("local_pointcloud", 1, rcvPointCloudCallBack);
  _pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallBack);

  _traj_pub =
      nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
  _traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
  _traj_vis_pub_r = nh.advertise<visualization_msgs::Marker>("vis_trajectory_r", 1);
  _path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_path", 1);
  _path_vis_pub_r = nh.advertise<visualization_msgs::Marker>("vis_path_r", 1);

  // set the obstacle map
  _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0;
  _map_upper << +_x_size / 2.0, +_y_size / 2.0, _z_size;
  _inv_resolution = 1.0 / _resolution;
  _max_x_id = (int)(_x_size * _inv_resolution);
  _max_y_id = (int)(_y_size * _inv_resolution);
  _max_z_id = (int)(_z_size * _inv_resolution);

  _astar_path_finder = new AstarPathFinder();
  _astar_path_finder->initGridMap(_resolution, _map_lower, _map_upper,
                                  _max_x_id, _max_y_id, _max_z_id);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
