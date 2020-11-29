#ifndef _NODE_UTILS_H_
#define _NODE_UTILS_H_

#include "poly_traj_utils/traj_utils.hpp"
#include <Eigen/Eigen>

using std::pair;
using std::vector;
using std::list;

typedef Eigen::Matrix<double, 9, 1> StatePVA;
typedef Eigen::Matrix<double, 6, 1> StatePV;
typedef Eigen::Matrix<double, 3, 1> Control;
typedef pair<double, double> BOUND;
typedef vector< BOUND > BOUNDS;

struct RRTNode {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  RRTNode* parent;
  StatePVA x;
  double cost_from_start;
  double tau_from_start;
  Piece poly_seg;
  list<RRTNode*> children;
  RRTNode(): parent(NULL), cost_from_start(DBL_MAX), tau_from_start(DBL_MAX){};
};

typedef RRTNode* RRTNodePtr;
typedef vector<RRTNodePtr, Eigen::aligned_allocator<RRTNodePtr>> RRTNodePtrVector;
typedef vector<RRTNode, Eigen::aligned_allocator<RRTNode>> RRTNodeVector;

#endif