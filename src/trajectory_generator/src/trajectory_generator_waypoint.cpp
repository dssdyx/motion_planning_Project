#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

/*
Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int n_seg = Time.size();
  MatrixXd PolyCoeff(n_seg, 3 * p_num1d);
  VectorXd Px(p_num1d*n_seg),Py(p_num1d*n_seg),Pz(p_num1d*n_seg);
  const static auto Factorial = [](int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };
  //compute Q M Ct define dF to compute dP  p = M'd
  //ROS_INFO("compute M");

  MatrixXd coeff_M(d_order, p_num1d);
  if(d_order == 4){
          coeff_M << 1,  1,  1,  1,  1,  1,  1,  1,
                   0,  1,  2,  3,  4,  5,  6,  7,
                   0,  0,  2,  6,  12, 20, 30, 42,
                   0,  0,  0,  6,  24, 60, 120,210;
   }
   else if(d_order == 3){
          coeff_M << 1,  1,  1,  1,  1,  1,
                   0,  1,  2,  3,  4,  5,
                   0,  0,  2,  6,  12, 20;

    }
    else{
          cout << "This derivatieve order is not provided getM!!!" << endl;
    }

  MatrixXd Mj;
  //MatrixXd M = MatrixXd::Zero(2*d_order*n_seg,2*d_order*n_seg);
  MatrixXd M = MatrixXd::Zero(p_num1d*n_seg,p_num1d*n_seg);
  for(int j =0; j< n_seg;j++)
  {
      Mj= MatrixXd::Zero(p_num1d,p_num1d);
      for(int k=0; k < d_order;k++)
      {
          Mj(k,k) = coeff_M(k,k);
          for(int i=k; i<p_num1d;i++)
          {
              if(k == i)
                  Mj(k+d_order,i) = Factorial(i);
              else
                  Mj(k + d_order , i) = Factorial(i)/Factorial(i-k)*pow(Time(j),i-k);
          }
      }
      M.block(j*p_num1d,j*p_num1d,p_num1d,p_num1d) = Mj;
  }
  //cout<<"M: "<<endl<<M<<endl;
  //cout<<"M inv: "<<endl<<M.inverse()<<endl;

  MatrixXd Qj;
  MatrixXd Q = MatrixXd::Zero(n_seg*p_num1d,n_seg*p_num1d);
  for(int j = 0; j<n_seg;j++)
  {
      Qj = MatrixXd::Zero(p_num1d,p_num1d);
      //ROS_INFO("Qj ready:%d",j);

      for(int i =d_order; i < p_num1d; i++)
    {
        for(int l =d_order; l< p_num1d;l++)
        {
            Qj(i,l) =
                    Factorial(i)/Factorial(i-d_order)
                    *Factorial(l)/Factorial(l-d_order)/(i+l-2*d_order+1)
                    *pow(Time(j),(i+l-2*d_order+1));
            //ROS_INFO("Qj get:%d",j);
        }
    }
      Q.block(j*p_num1d,j*p_num1d,p_num1d,p_num1d)= Qj;
      //ROS_INFO("Q ++:%d",j);
  }
  //cout<<"Q: "<<endl<<Q<<endl;

  int num_dF = 2*d_order + n_seg -1;
  int num_dP = 2*(n_seg -1);
  int num_dall = 2*d_order*n_seg;
  MatrixXd Ct = MatrixXd::Zero(num_dall,num_dF + num_dP);
  // head
  Ct(0,0) = 1;
  Ct(1,1) = 1;
  Ct(2,2) = 1;
  //end
  Ct(n_seg*2*d_order-3,3+n_seg-1) = 1;
  Ct(n_seg*2*d_order-2,3+n_seg) = 1;
  Ct(n_seg*2*d_order-1,3+n_seg+1) = 1;
  //pj middle
  for(size_t j =0;j<n_seg-1;j++)
  {
      Ct(3 + j*2*d_order,3+j) = 1;
      Ct(3 + j*2*d_order + 3,3+j) = 1;
  }
  //vj aj middle
  for(int j =0;j<n_seg-1;j++)
  {
      //vj
      Ct(3 + j*2*d_order + 1,num_dF + j*2) = 1;
      Ct(3 + j*2*d_order + 4,num_dF + j*2) = 1;
      //aj
      Ct(3 + j*2*d_order + 2,num_dF + j*2+1) = 1;
      Ct(3 + j*2*d_order + 5,num_dF + j*2+1) = 1;
  }

  //ROS_INFO("get Ct");
  //ROS_INFO("define dF");
  //cout<<"Ct: "<<endl<<Ct<<endl;

  VectorXd dF_x = VectorXd::Zero(num_dF);
  VectorXd dF_y = VectorXd::Zero(num_dF);
  VectorXd dF_z = VectorXd::Zero(num_dF);
  //px = path(j,0)
  // head
  dF_x(0) = Path(0,0);
  dF_x(1) = Vel(0,0);
  dF_x(2) = Acc(0,0);

  dF_y(0) = Path(0,1);
  dF_y(1) = Vel(0,1);
  dF_y(2) = Acc(0,1);

  dF_z(0) = Path(0,2);
  dF_z(1) = Vel(0,2);
  dF_z(2) = Acc(0,2);
  //end
  dF_x(num_dF - 3) = Path(n_seg,0);
  dF_x(num_dF - 2) = Vel(1,0);
  dF_x(num_dF - 1) = Acc(1,0);

  dF_y(num_dF - 3) = Path(n_seg,1);
  dF_y(num_dF - 2) = Vel(1,1);
  dF_y(num_dF - 1) = Acc(1,1);

  dF_z(num_dF - 3) = Path(n_seg,2);
  dF_z(num_dF - 2) = Vel(1,2);
  dF_z(num_dF - 1) = Acc(1,2);
  //pj middle
  for(int j =0; j<n_seg-1;j++)
  {
      dF_x(3+j) = Path(j+1,0);

      dF_y(3+j) = Path(j+1,1);

      dF_z(3+j) = Path(j+1,2);
  }

  MatrixXd R,RFP,RPP;
  R = Ct.transpose()*M.transpose().inverse()*Q*M.inverse()*Ct;
  RFP = R.block(0,num_dF,num_dF,num_dP);
  RPP = R.block(num_dF,num_dF,num_dP,num_dP);
  VectorXd dP_x = - RPP.inverse()*RFP.transpose()*dF_x;
  VectorXd dP_y = - RPP.inverse()*RFP.transpose()*dF_y;
  VectorXd dP_z = - RPP.inverse()*RFP.transpose()*dF_z;

  VectorXd dFP_x = VectorXd::Zero(num_dF+num_dP);
  VectorXd dFP_y = VectorXd::Zero(num_dF+num_dP);
  VectorXd dFP_z = VectorXd::Zero(num_dF+num_dP);
  dFP_x << dF_x,dP_x;
  dFP_y << dF_y,dP_y;
  dFP_z << dF_z,dP_z;
  VectorXd dALL_x = VectorXd::Zero(num_dall);
  VectorXd dALL_y = VectorXd::Zero(num_dall);
  VectorXd dALL_z = VectorXd::Zero(num_dall);
  dALL_x = Ct*dFP_x;
  dALL_y = Ct*dFP_y;
  dALL_z = Ct*dFP_z;

  Px = M.inverse()*dALL_x;
  Py = M.inverse()*dALL_y;
  Pz = M.inverse()*dALL_z;
  //cout<<"dALL_x: "<<endl<<dALL_x<<endl;
  //cout<<"dALL_y: "<<endl<<dALL_y<<endl;
  //cout<<"dALL_z: "<<endl<<dALL_z<<endl;
  //cout<<"M-1: "<<endl<<M.inverse()<<endl;

  for(int j =0;j<n_seg;j++)
  {
      //PolyCoeff.row(j) << Px,Py,Pz;
      PolyCoeff.block(j, 0, 1, p_num1d) = Px.segment(j * p_num1d, p_num1d).transpose();
      PolyCoeff.block(j, p_num1d, 1, p_num1d) = Py.segment(j * p_num1d, p_num1d).transpose();
      PolyCoeff.block(j, 2*p_num1d, 1, p_num1d) = Pz.segment(j * p_num1d, p_num1d).transpose();
  }
  //cout << "X PolyCoeff.block(0, dim*p_num1d, n_segment, p_num1d) :" << endl
  //            << PolyCoeff.block(0, 0*p_num1d, n_seg, p_num1d) << endl;
  //cout << "Y PolyCoeff.block(0, dim*p_num1d, n_segment, p_num1d) :" << endl
  //            << PolyCoeff.block(0, 1*p_num1d, n_seg, p_num1d) << endl;
 // cout << "Z PolyCoeff.block(0, dim*p_num1d, n_segment, p_num1d) :" << endl
  //            << PolyCoeff.block(0, 2*p_num1d, n_seg, p_num1d) << endl;
  //ROS_INFO("get p");


  return PolyCoeff;
}
*/
Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and acceleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    int n_segment = Time.size();                              // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(n_segment, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    VectorXd Px(p_num1d * n_segment), Py(p_num1d * n_segment), Pz(p_num1d * n_segment);



    // get Q
    MatrixXd Q = getQ(n_segment, d_order,  p_num1d, Time);
    //cout << "Matrix Q is:\n"<< endl << Q << endl;

    /* Produce Mapping Matrix  to the entire trajectory,
     * M is a mapping matrix that maps polynomial coefficients to derivatives.   */
    MatrixXd M = getM(n_segment, d_order,  p_num1d, Time);
    //cout << "Mapping matrix M is:\n" << endl << M << endl;


    // compute Ct
    MatrixXd Ct = getCt(n_segment, d_order);
    //cout <<"Ct: \n"<< endl << Ct << endl;

    MatrixXd C = Ct.transpose();
    //cout << "C is:\n" << endl << C << endl;

    MatrixXd M_inv = M.inverse();
    MatrixXd M_inv_t = M_inv.transpose();

    //cout << "M_inv is:" << endl << M_inv << endl;
    //cout << "M_inv_transp is:" << endl << M_inv_t << endl;
    //cout << "size of C is: " << C.rows() << ", " << C.cols() << endl;
    //cout << "size of M_inv_t is: " << M_inv_t.rows()  << ", " <<  M_inv_t.cols() << endl;
    //cout << "size of Q  is: " << Q.rows()  << ", " << Q.cols()  << endl;
    //cout << "size of M_inv  is: " << M_inv.cols() << ", " << M_inv.rows() << endl;
    //cout << "size of Ct  is: " << Ct.rows() << ", " << Ct.cols() << endl;

    MatrixXd R = C * M_inv_t * Q * M_inv * Ct; // M is not changed
    //cout << "R is:\n" << endl << C << endl;

    int num_d_F = 2 * d_order + n_segment - 1;
    int num_d_P = (n_segment - 1) * (d_order - 1);

    MatrixXd R_pp = R.bottomRightCorner(num_d_P, num_d_P);
    //cout << "R_pp is:\n" << endl << C << endl;

    MatrixXd R_fp = R.topRightCorner(num_d_F, num_d_P);
    //cout << "R_fp is:\n" << endl << C << endl;



    // STEP 3: compute dF for x, y, z respectively
    for(int dim = 0; dim < 3; dim++){
        VectorXd wayPoints = Path.col(dim);
        //cout << "waypoints: " << endl << wayPoints << endl;
        VectorXd d_F = VectorXd::Zero(num_d_F);

        d_F(0) = wayPoints(0); //p0
        // v0,0 a0,0 ... are 0

        // pT,0, pT,1, ,,PT,n_seg-2
        for(int i = 0; i < n_segment - 1; i++ ){
            d_F(d_order + i) = wayPoints(i + 1);
        }
        // pT,n_seg-1
        d_F(d_order + n_segment - 1) = wayPoints(n_segment);
        //cout << "d_F is:" << endl << d_F << endl;

        VectorXd d_P = -1.0 * R_pp.inverse() * R_fp.transpose() * d_F;
        //cout << "d_P is:" << endl << d_P << endl;

        VectorXd d_total(d_F.rows() + d_P.rows());
        d_total << d_F, d_P;
        //cout << "d_total is:" << endl << d_total << endl;

        VectorXd poly_coef_1d = M.inverse() * Ct * d_total;
        //cout<< "Dimension " << dim <<" coefficients: "<< endl << poly_coef_1d << endl;

        //cout << "PolyCoeff.block(0, dim*p_num1d, n_segment, p_num1d) :" << endl
        //    << PolyCoeff.block(0, dim*p_num1d, n_segment, p_num1d) << endl;
        MatrixXd poly_coef_1d_t = poly_coef_1d.transpose();
        //cout << "poly_coef_1d.transpose() :" << endl << poly_coef_1d_t << endl;
        // PolyCoeff(n_segment, 3 * p_num1d)
        for(int k = 0; k < n_segment; k++){
            PolyCoeff.block(k, dim*p_num1d, 1, p_num1d) = poly_coef_1d_t.block(0,k*p_num1d, 1, p_num1d);
        }

    }
    //cout << "PolyCoeff :" << endl << PolyCoeff << endl;
    return PolyCoeff;
}
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}
MatrixXd TrajectoryGeneratorWaypoint::getCt(int const n_seg, int const d_order){ // d_order is 4
    /*
     * get Selection Matrix
     * args:
     *      n_seg: the number of segments
     *      d_order: the deferential order: if minimal jerk. it is 3,
     * return:
     *      Ct: a matrix,
     *      row corresponds to original variable vector d ( 2 * d_order * n_seg )
     *      column corresponds to [ dF, dP ]' ( d_order*2*n_seg - (n_seg-1)*d_order )
     *      Note: the variables implicitly eliminates same variables""
     */
    // we have n_seg segments, from 0 to n_seg-1
    int ct_rows = d_order*2*n_seg;
    int ct_cols = d_order*2*n_seg - (n_seg-1)*d_order;
    MatrixXd Ct = MatrixXd::Zero(ct_rows, ct_cols);
    vector<int> d_vector;
    for(int k = 0; k < n_seg; k ++){
        for(int t = 0; t < 2; t++){
            for(int d = 0; d < d_order; d++){
                d_vector.push_back(k*100+t*10+d);
            }
        }
    }
    int val, row;
    int col = 0; // column of one in Ct

    // fixed starting point at segment 0 in [ dF, dP ]'
    int k = 0;
    int t = 0;
    int d = 0;
    for(d = 0; d < d_order; d++){
        val = k * 100 + t * 10 + d;
        auto it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;
        col += 1;
    }
    // fixed final point at segment 0, 1, 2, n_seg-2 in [ dF, dP ]'
    t = 1;
    d = 0;
    for(k = 0; k < n_seg - 1; k++){
        val = k * 100 + t * 10 + d;
        auto it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;

        val = (k + 1) * 100 + (t - 1) * 10 + d;
        it= std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;

        col += 1;
    }

    // fixed final point at segment n_seg-1 in [ dF, dP ]'
    k = n_seg - 1;
    t = 1;
    d = 0;
    for(d = 0; d < d_order; d++){
        val = k * 100 + t * 10 + d;
        auto it = std::find(d_vector.begin(), d_vector.end(), val);
        row = std::distance(d_vector.begin(), it);
        Ct(row, col) = 1;
        col += 1;
    }

    // free variables at segment 0, 1, 2, n_seg-1 in [ dF, dP ]'
    k = 0;
    t = 1;
    d = 1;
    for(k = 0; k < n_seg - 1; k++){
        for(d = 1; d < d_order; d++){
            val = k * 100 + t * 10 + d;
            auto it = std::find(d_vector.begin(), d_vector.end(), val);
            row = std::distance(d_vector.begin(), it);
            Ct(row, col) = 1;

            val = (k + 1) * 100 + (t - 1) * 10 + d;
            it = std::find(d_vector.begin(), d_vector.end(), val);
            row = std::distance(d_vector.begin(), it);
            Ct(row, col) = 1;

            col += 1;
        }

    }
    return Ct;
}
MatrixXd TrajectoryGeneratorWaypoint::getM(int const n_seg, int const d_order, int const p_num1d, const Eigen::VectorXd &ts){
    /*
     * get Mapping Matrix
     * args:
     *      n_seg: the number of segments
     *      d_order: the deferential order: if minimal jerk. it is 3,
     *      p_num1d: the number of variables in each segment, if minimal jerk, it is 6
     *      ts: time allocation as a column vector, size: n_seg x 1,
     * return:
     *      M: a matrix, size: n_seg * p_num1d x n_seg * p_num1d
     */
//    MatrixXd M, M_temp, zeros;
    MatrixXd M = MatrixXd::Zero(n_seg * p_num1d, n_seg * p_num1d);
    MatrixXd coeff(d_order, p_num1d);
    if(d_order == 4){
        coeff << 1,  1,  1,  1,  1,  1,  1,  1,
                 0,  1,  2,  3,  4,  5,  6,  7,
                 0,  0,  2,  6,  12, 20, 30, 42,
                 0,  0,  0,  6,  24, 60, 120,210;
    }
    else if(d_order == 3){
        coeff << 1,  1,  1,  1,  1,  1,
                 0,  1,  2,  3,  4,  5,
                 0,  0,  2,  6,  12, 20;

    }
    else{
        cout << "This derivatieve order is not provided getM!!!" << endl;
    }


    double t;
    for(int k = 0; k < n_seg; k++){
        // calculate M_k of the k-th segment
        MatrixXd M_k = MatrixXd::Zero(p_num1d, p_num1d); // Matrix size: p_num1d x p_num1d
        // time of the k-th segment
        t = ts(k);
        for(int i = 0; i < d_order; i++){
            M_k(i, i) = coeff(i, i);
        }

        for(int i = 0; i < d_order; i++){
            for(int j = i; j < p_num1d; j++){
                if( i == j){
                    M_k(i+d_order, j) = coeff(i, j) ;
                }
                else{
                    M_k(i+d_order, j) = coeff(i, j) * pow(t, j - i);
                }
            }
        }




        M.block(k*p_num1d, k*p_num1d, p_num1d, p_num1d) = M_k;
    }
    return M;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::getQ(int const n_seg, int const d_order, int const p_num1d, const Eigen::VectorXd &ts){
    /*
     * get Q matrix
     * args:
     *      n_seg: the number of segments
     *      d_order: the deferential order: if minimal snap. it is 4
     *      p_num1d: the number of variables in each segment
     *      ts: time allocation as a column vector, size: n_seg x 1,
     * return:
     *      Q: a matrix, size: n_seg * p_num1d x n_seg * p_num1d
     *      Note: p = [p0, p1, p2,...pn-1]'
     */

    MatrixXd Q = MatrixXd::Zero(n_seg * p_num1d, n_seg * p_num1d);
    for(int k = 0; k < n_seg; k++) {
        Eigen::MatrixXd Q_k = Eigen::MatrixXd::Zero(p_num1d, p_num1d); // Matrix size: p_num1d x p_num1d
        for (int i = 0; i < p_num1d; i++) {
            for (int j = 0; j < p_num1d; j++) {
                if (i < d_order || j < d_order)
                    continue;
                else
                    Q_k(i, j) = 1.0 * Factorial(i) / Factorial(i - d_order) * Factorial(j) /
                                    Factorial(j - d_order) /
                                    (i + j - d_order * 2 + 1) * pow(ts(k), i + j - 2 * d_order + 1);
            }

        }
        Q.block(k * p_num1d, k * p_num1d, p_num1d, p_num1d) = Q_k;
    }
    return Q;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
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

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}
