#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id) {
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

  GridNodeMap = new GridNodePtr **[GLX_SIZE];
  for (int i = 0; i < GLX_SIZE; i++) {
    GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
    for (int j = 0; j < GLY_SIZE; j++) {
      GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
      for (int k = 0; k < GLZ_SIZE; k++) {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
      }
    }
  }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr) {
  ptr->id = 0;
  ptr->cameFrom = NULL;
  ptr->gScore = inf;
  ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids() {
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++)
        resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y,
                             const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  if (idx_x == 0 || idx_y == 0 || idx_z == GLZ_SIZE || idx_x == GLX_SIZE ||
      idx_y == GLY_SIZE)
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
  else {
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
  }
}

vector<Vector3d> AstarPathFinder::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++) {
        // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (GridNodeMap[i][j][k]->id ==
            -1) // visualize nodes in close list only
          visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const {
  return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y,
                                        const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y,
                                    const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr,
                                          vector<GridNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector3i neighborIdx;
  for (int dx = -1; dx < 2; dx++) {
    for (int dy = -1; dy < 2; dy++) {

        if (dx == 0 && dy == 0)
          continue;


        neighborIdx(0) = (currentPtr->index)(0) + dx;
        neighborIdx(1) = (currentPtr->index)(1) + dy;
        neighborIdx(2) = (currentPtr->index)(2);
        if(isOccupied(neighborIdx)) continue;
        if( dx*dx+dy*dy == 2 && (isOccupied(currentPtr->index(0)+dx,currentPtr->index(1),currentPtr->index(2))
                || isOccupied(currentPtr->index(0),currentPtr->index(1)+dy,currentPtr->index(2))))
            continue;

        if (neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE ||
            neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE ||
            neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE) {
          continue;
        }

        neighborPtrSets.push_back(
            GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
        edgeCostSets.push_back(sqrt(dx * dx + dy * dy ));
    }
  }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
  // using digonal distance and one type of tie_breaker.
  double h;
  int dx,dy,dz;
  //use diagonal heu
  dx = abs(node1->index(0) - node2->index(0));
  dy = abs(node1->index(1) - node2->index(1));
  //dz = abs(node1->index(2) - node2->index(2));
  //dz = 0;
 /*
  int mind = min(min(dx, dy), dz);
  int maxd = max(max(dx, dy), dz);
  int midd = (dx + dy + dz) - (mind + maxd);
  h = sqrt(3)*mind + sqrt(2) * (midd-mind) + (maxd-mind);
  */
  h = (dx + dy) + (sqrt(2) - 2) *min(dx,dy);
  h = 1.0 * (1 + tie_breaker_) * h;

  return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();
  resetUsedGrids();

  // index of start_point and end_point
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;


  // position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  GridNodePtr startPtr = new GridNode(start_idx, start_pt);
  GridNodePtr endPtr = new GridNode(end_idx, end_pt);

  // openSet is the open_list implemented through multimap in STL library
  openSet.clear();
  // currentPtr represents the node with lowest f(n) in the open_list
  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  // put start node in open set
  startPtr->gScore = 0;
  /**
   *
   * STEP 1.1:  finish the AstarPathFinder::getHeu
   *
   * **/
  startPtr->fScore = getHeu(startPtr, endPtr);

  startPtr->id = 1;
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));

  /**
   *
   * STEP 1.2:  some else preparatory works which should be done before while
   * loop
   *
   * **/
  terminatePtr = NULL;

  double tentative_gScore;
  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;

  /**
   *
   * STEP 1.3:  finish the loop
   *
   * **/
  //cout<<"openset size: "<<openSet.size()<<endl;
  while (!openSet.empty()) {
      currentPtr = openSet.begin()->second;
      currentPtr->id = -1;
      openSet.erase(openSet.begin());
      if(currentPtr->index == goalIdx){
          ros::Time time_2 = ros::Time::now();
          terminatePtr = currentPtr;
          ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );
          return;
      }
      AstarGetSucc(currentPtr,neighborPtrSets,edgeCostSets);

      //ROS_INFO("*********");
      //cout<<"a star neigh size: "<<neighborPtrSets.size()<<endl;
      int count_closed = 0;
      for(size_t i=0;i<neighborPtrSets.size();i++)
      {
          neighborPtr = neighborPtrSets[i];
          //cout<<"neighbor id: "<<neighborPtr->id<<endl;
          if(neighborPtr->id == 0){
              neighborPtr->id = 1;
              neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
              neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr);
              neighborPtr->cameFrom = currentPtr;
              openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));
              continue;
          }
          else if(neighborPtr->id == 1){
              double tmp_g = currentPtr -> gScore + edgeCostSets[i];
              if(neighborPtr->gScore > tmp_g)
              {
                  neighborPtr->gScore = tmp_g;
                  neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr);
                  neighborPtr->cameFrom = currentPtr;
              }
              continue;
          }
          else{
              count_closed++;
              continue;
          }
      }
      //cout<<"a star openset size: "<<openSet.size()<<endl;
      if(count_closed == neighborPtrSets.size())
          cout<<"neighbour all closed"<<endl;
  }
  // if search fails
  ros::Time time_2 = ros::Time::now();
  //if ((time_2 - time_1).toSec() > 0.1)
  ROS_WARN("A_star fail !");
}

vector<Vector3d> AstarPathFinder::getPath() {
  vector<Vector3d> path;
  vector<GridNodePtr> gridPath;

  /**
   *
   * STEP 1.4:  trace back the found path
   *
   * **/
  if(terminatePtr == NULL) return path;
  gridPath.push_back(terminatePtr);
  auto lastnodePtr = terminatePtr->cameFrom;
  while(1)
  {
      gridPath.push_back(lastnodePtr);
      lastnodePtr = lastnodePtr->cameFrom;
      if(!lastnodePtr)
          break;
  }

  for (auto ptr: gridPath)
      path.push_back(ptr->coord);

  reverse(path.begin(),path.end());

  return path;
}

vector<Vector3d> AstarPathFinder::pathSimplify(const vector<Vector3d> &path,
                                               double path_resolution) {
  vector<Vector3d> subPath;
  /**
   *
   * STEP 2.1:  implement the RDP algorithm
   *
   * **/
  double dmax = 0;
  int index = 0;
  int end = path.size();
  if(end < 3) return path;
  for(size_t i = 1; i < end - 1; i++)
  {
      //double d = perpendicularDistance(path[i], path.front(),path.back());
      //cout<<"dis: "<<d<<endl;

      Vector3d p = path.back()-path.front();
      Vector3d pp = path[i] - path.front();
      double d = (pp.cross(p)).norm()/p.norm();
      if(d > dmax)
      {
          index = i;
          dmax = d;
          /*
          cout<<"d: "<<d<<endl;
          cout<<"p:    "<<p(0)<<" "<<p(1)<<" "<<p(2)<<endl;
          cout<<"pp:   "<<pp(0)<<" "<<pp(1)<<" "<<pp(2)<<endl;
          cout<<"head: "<<path.front()(0)<<" "<<path.front()(1)<<" "<<path.front()(2)<<endl;
          cout<<"poin: "<<path[i](0)<<" "<<path[i](1)<<" "<<path[i](2)<<endl;
          cout<<"end:  "<<path.back()(0)<<" "<<path.back()(1)<<" "<<path.back()(2)<<endl;
          */
       }
  }
  cout<<"dmax: "<<dmax<<endl;
  cout<<"index: "<<index<<endl;
  if(dmax > path_resolution){
      vector<Vector3d> path1(path.begin(),path.begin()+index+1);
      vector<Vector3d> path2(path.begin()+index,path.end());


      //path1.insert(path1.end(),path.begin(),path.begin()+index);
      //path2.insert(path2.end(),path.begin()+index+1,path.end());
      //cout<<"path size "<<path.size()<<endl;
      cout<<"path1 size "<<path1.size()<<endl;
      cout<<"path2 size "<<path2.size()<<endl;
      vector<Vector3d> Result_list1 = pathSimplify(path1,path_resolution);
      vector<Vector3d> Result_list2 = pathSimplify(path2,path_resolution);
      //cout<<"result get"<<endl;
      //subPath.insert(subPath.end(),Result_list1.begin(),Result_list1.end());
      //subPath.insert(subPath.end(),Result_list2.begin(),Result_list2.end());
      subPath = Result_list1;
      subPath.pop_back();
      subPath.insert(subPath.end(),Result_list2.begin(),Result_list2.end());
  }
  else{
      subPath.push_back(path.front());
      subPath.push_back(path.back());
      //cout<<"subpath size: "<<subPath.size()<<endl;
  }

  return subPath;
}
double AstarPathFinder::perpendicularDistance(const Eigen::Vector3d &C, const Eigen::Vector3d &A, const Eigen::Vector3d &B)
{

}

Vector3d AstarPathFinder::getPosPoly(MatrixXd polyCoeff, int k, double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

int AstarPathFinder::safeCheck(MatrixXd polyCoeff, VectorXd time) {
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe
  /**
   *
   * STEP 3.3:  finish the sareCheck()
   *
   * **/
  //for each segment ,set time interval and iteratively check
  const double time_interval = 0.01;
  for(size_t j = 0;j<polyCoeff.rows();j++)
  {
      if(unsafe_segment != -1) break;
      for(double t = 0.0;t<time(j);t+=time_interval)
      {
          Vector3d pos = getPosPoly(polyCoeff,j,t);
          Vector3d start_pos = getPosPoly(polyCoeff,j,0);
          Vector3d end_pos = getPosPoly(polyCoeff,j,time(j));
          Vector3i idx = coord2gridIndex(pos);
          if(isOccupied(idx))
          {
              //cout<<"collision pos: "<<endl<<pos<<endl;
              cout<<"collision idx: "<<endl<<idx<<endl;
              unsafe_segment = j;
              break;
          }
      }
  }

  return unsafe_segment;
}
