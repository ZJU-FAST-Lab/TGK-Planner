#ifndef _NODE_UTILS_H_
#define _NODE_UTILS_H_

#include "poly_traj_utils/traj_utils.hpp"
#include <Eigen/Eigen>

using std::pair;
using std::vector;
using std::list;

#define CLOSED 'a'
#define OPEN 'b'
#define UNVISITED 'c'

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


struct FMTNode 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  FMTNode* parent;
  StatePVA x;
  double g_score; //cost from start
  double f_score; //g_score + cost_to_go (heuristic estimation)
  Piece poly_seg;
  list<FMTNode*> children;
  char status;
  FMTNode(): parent(NULL), g_score(DBL_MAX), f_score(DBL_MAX), poly_seg(), status(UNVISITED) {};
};
typedef FMTNode* FMTNodePtr;
typedef vector<FMTNodePtr, Eigen::aligned_allocator<FMTNodePtr>> FMTNodePtrVector;
typedef vector<FMTNode, Eigen::aligned_allocator<FMTNode>> FMTNodeVector;

class FMTNodeComparator 
{
public:
  bool operator()(FMTNodePtr node1, FMTNodePtr node2) 
  {
    return node1->f_score > node2->f_score;
  }
};

#endif