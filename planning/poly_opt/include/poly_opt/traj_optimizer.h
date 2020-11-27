#ifndef _TRAJ_OPTIMIZER_H_
#define _TRAJ_OPTIMIZER_H_

#include "occ_grid/pos_checker.h"
#include "visualization_utils/visualization_utils.h"
#include <Eigen/Eigen>
#include <ros/ros.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace tgk_planner 
{
class TrajOptimizer
{
public:
  TrajOptimizer(const ros::NodeHandle &node);
  TrajOptimizer();
  void setPosChecker(const PosChecker::Ptr& checker)
  {
    pos_checker_ptr_ = checker;
  };
  void getTraj(Trajectory &traj)
  {
    traj = optimized_traj_;
  };
  bool setFrontEndTraj(const Trajectory &traj)
  {
    front_end_traj_ = traj;
    return initialize();
  };
  void setVisualizer(const VisualRviz::Ptr &vis)
  {
    vis_ptr_ = vis;
  }
  bool solve();

  typedef shared_ptr<TrajOptimizer> Ptr;

private:
  PosChecker::Ptr pos_checker_ptr_;
  VisualRviz::Ptr vis_ptr_;

  /** coefficient of polynomials*/
  Eigen::MatrixXd coeff_;  
  Eigen::MatrixXd coeff0_;
  Trajectory optimized_traj_, front_end_traj_;
  
  /** way points info, from start point to end point **/
  std::set<int> fixed_pos_;
  Eigen::MatrixXd path_;
  Eigen::MatrixXd vel_way_points_; 
  Eigen::MatrixXd acc_way_points_; 
  std::vector<double> time_;
  int m_; //segments number
  int n_; //fixed_pos number (exclude start and goal)
  
  /** important matrix and  variables*/
  Eigen::MatrixXd Ct_;
  Eigen::MatrixXd Z_, Zp_;
  Eigen::MatrixXd A_inv_multiply_Ct_;
  
  Eigen::MatrixXd A_;
  Eigen::MatrixXd A_inv_;
  Eigen::MatrixXd Q_smooth_, Q_close_, Q_acc_;
  Eigen::MatrixXd C_;
  Eigen::MatrixXd L_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd Rff_;
  Eigen::MatrixXd Rpp_;
  Eigen::MatrixXd Rpf_;
  Eigen::MatrixXd Rfp_;
  Eigen::VectorXd Dx_, Dy_, Dz_;
  
  Eigen::MatrixXd V_;
  Eigen::MatrixXd Df_;
  Eigen::MatrixXd Dp_;
  int num_dp;
  int num_df;
  int num_point;

  double vel_limit_, acc_limit_;
  bool initialize();
  bool checkTrajectoryConstraints(const Trajectory &traj);
  void setWayPointsAndTime(const vector<Eigen::Vector3d>& way_points, 
                           const vector<Eigen::Vector3d>& vel, 
                           const vector<Eigen::Vector3d>& acc, 
                           const std::vector<double> &time);
  void tryQPCloseForm(double percent_of_close, double percent_of_acc);
  void tryQPCloseForm(double percent_of_close);
  void tryQPCloseForm();

  void calMatrixA();
  void calMatrixCandMatrixZ(int type);
  void calMatrixC();
  void calMatrixQ_smooth(int type);
  void calMatrixQ_close();
  void calMatrixQ_acc_consistent();
  enum QSmoothType
  {
    MINIMUM_ACC, 
    MINIMUM_JERK, 
    MINIMUM_SNAP
  };
  enum CType
  {
    MIDDLE_P_V_CONSISTANT,
    MIDDLE_P_V_A_CONSISTANT
  };
};

}

#endif