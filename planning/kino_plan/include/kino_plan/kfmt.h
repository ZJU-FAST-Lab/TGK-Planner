#ifndef _KFMT_H_
#define _KFMT_H_

#include "node_utils.h"
#include "kdtree.h"
#include "visualization_utils/visualization_utils.h"
#include "occ_grid/pos_checker.h"
#include "poly_traj_utils/traj_utils.hpp"
#include "bvp_solver.h"
#include "bias_sampler.h"

#include <vector>
#include <stack>

#define FAILURE 0
#define SUCCESS 1
#define SUCCESS_CLOSE_GOAL 2

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector2i;
using Eigen::Vector3d;
using Eigen::Vector3i;
using Eigen::VectorXd;
using std::list;
using std::pair;
using std::stack;
using std::vector;

namespace tgk_planner
{

class KFMTPlanner
{
public:
  KFMTPlanner();
  KFMTPlanner(const ros::NodeHandle &nh);
  ~KFMTPlanner();

  // api
  void init(const ros::NodeHandle &nh);
  void setPosChecker(const PosChecker::Ptr &checker);
  void setVisualizer(const VisualRviz::Ptr &vis);
  int plan(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
           Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
           double search_time, const Vector3d &normal, const Vector3d &dire, bool need_consistancy);
  void getTraj(Trajectory &traj);

  typedef shared_ptr<KFMTPlanner> KFMTPlannerPtr;

private:
  void reset();
  int fmtStar(const StatePVA &x_init, const StatePVA &x_final, int sample_num);
  void batchSampling(int sample_num, kdtree *kd_tree);
  double dist(const StatePVA &x0, const StatePVA &x1);
  void fillTraj(const FMTNodePtr &goal_leaf, Trajectory& traj);
  void chooseBypass(FMTNodePtr &goal_leaf, const FMTNodePtr &tree_start_node);

  // vis
  bool debug_vis_;
  VisualRviz::Ptr vis_ptr_;
  void sampleWholeTree(const FMTNodePtr &root, vector<StatePVA> *vis_x, vector<Vector3d>& knots);

  FMTNodePtrVector node_pool_; //pre allocated in Constructor
  std::priority_queue<FMTNodePtr, vector<FMTNodePtr>, FMTNodeComparator> open_set_;
  int sampled_node_num_, iter_num_;
  Trajectory traj_;
  FMTNodePtr start_node_, goal_node_, close_goal_node_;

  // radius for for/backward search
  double getForwardRadius(double tau, double cost);
  double getBackwardRadius(double tau, double cost);
  struct kdres *getForwardNeighbour(const StatePVA &x1, struct kdtree *kd_tree, double tau, double radius_p);
  struct kdres *getBackwardNeighbour(const StatePVA &x1, struct kdtree *kd_tree, double tau, double radius_p);

  // nodehandle params
  double radius_cost_between_two_states_;
  double rho_, lambda_heu_, tie_breaker_;
  double v_mag_sample_;
  double vel_limit_, acc_limit_;
  bool allow_close_goal_;
  double search_time_;
  int preallocate_node_pool_num_, sample_num_per_meter_;

  // environment
  PosChecker::Ptr pos_checker_ptr_;
  bool checkSegmentConstraints(const Piece &seg);
  bool getTraversalLines(const Piece &seg, vector<pair<Vector3d, Vector3d>> &traversal_lines);

  // bvp_solver
  BVPSolver::IntegratorBVP bvp_;

  // bias_sampler
  BiasSampler sampler_;

  // evaluation
  void evaluateTraj(const Trajectory& traj, double &traj_duration, double &traj_length, int &seg_nums, double &acc_integral, double &jerk_integral);

  // for replan
  Vector3d replan_normal_, replan_dire_;
  bool need_consistancy_;
};

} // namespace tgk_planner

#endif //_KFMT_H_
