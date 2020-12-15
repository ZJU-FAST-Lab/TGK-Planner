#include "kino_plan/krrtplanner.h"
#include <queue>

#ifdef TIMING
double check_path_time = 0.0;
int check_path_nums = 0;
double check_cost_time = 0.0;
int check_cost_nums = 0;
double calculate_state_control_time = 0.0;
int calculate_state_control_nums = 0;
double kd_insert_time = 0.0;
int kd_insert_nums = 0;
double kd_query_time = 0.0;
int kd_query_nums = 0;
double linear_search_parent_time = 0.0;
int linear_search_parent_nums = 0;
double linear_rewire_time = 0.0;
int linear_rewire_nums = 0;
double kd_search_parent_time = 0.0;
int kd_search_parent_nums = 0;
double kd_rewire_time = 0.0;
int kd_rewire_nums = 0;
double state_time = 0.0;
double new_time = 0.0;
double cal_forward_bound_time = 0.0;
int cal_forward_bound_nums = 0;
double cal_backward_bound_time = 0.0;
int cal_backward_bound_nums = 0;
double connect_time = 0.0;
int connect_nums = 0;
double solve_poly_time = 0.0;
int solve_poly_nums = 0;
#endif

namespace tgk_planner
{
KRRTPlanner::KRRTPlanner(const ros::NodeHandle& nh): sampler_(nh)
{
}

KRRTPlanner::~KRRTPlanner()
{
  for (int i = 0; i < tree_node_nums_; i++) {
    delete start_tree_[i];
  }
}

void KRRTPlanner::init(const ros::NodeHandle& nh)
{
  nh.param("krrt/vel_limit", vel_limit_, -1.0);
  nh.param("krrt/acc_limit", acc_limit_, -1.0);
  nh.param("krrt/debug_vis", debug_vis_, false);
  nh.param("krrt/rho", rho_, -1.0);
  nh.param("krrt/tree_node_nums", tree_node_nums_, 0);
  nh.param("krrt/orphan_nums", orphan_nums_, 0);
  nh.param("krrt/radius_cost_between_two_states", radius_cost_between_two_states_, 0.0);
  nh.param("krrt/allow_orphan", allow_orphan_, false);
  nh.param("krrt/allow_close_goal", allow_close_goal_, false);
  nh.param("krrt/stop_after_first_traj_found", stop_after_first_traj_found_, false);
  nh.param("krrt/rewire", rewire_, true);
  ROS_WARN_STREAM("[krrt] param: vel_limit: " << vel_limit_);
  ROS_WARN_STREAM("[krrt] param: acc_limit: " << acc_limit_);
  ROS_WARN_STREAM("[krrt] param: debug_vis: " << debug_vis_);
  ROS_WARN_STREAM("[krrt] param: rho: " << rho_);
  ROS_WARN_STREAM("[krrt] param: tree_node_nums: " << tree_node_nums_);
  ROS_WARN_STREAM("[krrt] param: orphan_nums: " << orphan_nums_);
  ROS_WARN_STREAM("[krrt] param: radius_cost_between_two_states: " << radius_cost_between_two_states_);
  ROS_WARN_STREAM("[krrt] param: allow_orphan: " << allow_orphan_);
  ROS_WARN_STREAM("[krrt] param: allow_close_goal: " << allow_close_goal_);
  ROS_WARN_STREAM("[krrt] param: stop_after_first_traj_found: " << stop_after_first_traj_found_);
  ROS_WARN_STREAM("[krrt] param: rewire: " << rewire_);

  bvp_.init(TRIPLE_INTEGRATOR);
  bvp_.setRho(rho_);

  valid_start_tree_node_nums_ = 0;
  
  //pre allocate memory
  start_tree_.resize(tree_node_nums_);
  for (int i = 0; i < tree_node_nums_; i++) 
  {
    start_tree_[i] = new RRTNode;
  }
}

void KRRTPlanner::setPosChecker(const PosChecker::Ptr &checker)
{
  pos_checker_ptr_ = checker;
  sampler_.setPosChecker(checker);
}

void KRRTPlanner::setVisualizer(const VisualRviz::Ptr &vis)
{
  vis_ptr_ = vis;
}

// reset() is called every time before plan();
void KRRTPlanner::reset()
{
  for (int i = 0; i <= valid_start_tree_node_nums_; i++) 
  {
    start_tree_[i]->parent = nullptr;
    start_tree_[i]->children.clear();
  }
  
  if (allow_orphan_) 
  {
    orphans_.clear();
  }
  
  valid_start_tree_node_nums_ = 0;
}

int KRRTPlanner::plan(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc, 
                      Vector3d end_pos, Vector3d end_vel, Vector3d end_acc, 
                      double search_time, const Vector3d &normal, const Vector3d &dire, bool need_consistancy)
{
  ros::Time plan_start_time = ros::Time::now();
  reset();
  if (pos_checker_ptr_->getVoxelState(start_pos) != 0) 
  {
    ROS_ERROR("[KRRT]: Start pos collide or out of bound");
    return FAILURE;
  }
  
  if (!pos_checker_ptr_->validatePosSurround(end_pos)) 
  {
    Vector3d shift[20] = {Vector3d(-1.0,0.0,0.0), Vector3d(0.0,1.0,0.0), Vector3d(0.0,-1.0,0.0), 
                         Vector3d(0.0,0.0,1.0), Vector3d(0.0,0.0,-1.0), Vector3d(1.0,0.0,0.0), 
                         Vector3d(-2.0,0.0,0.0), Vector3d(0.0,2.0,0.0), Vector3d(0.0,-2.0,0.0), 
                         Vector3d(0.0,0.0,2.0), Vector3d(0.0,0.0,-2.0), Vector3d(2.0,0.0,0.0), 

                         Vector3d(1.0,1.0,1.0), Vector3d(1.0,1.0,-1.0), Vector3d(1.0,-1.0,1.0), 
                         Vector3d(1.0,-1.0,-1.0), Vector3d(-1.0,1.0,-1.0), Vector3d(-1.1,1.0,1.0), 
                         Vector3d(-1.0,-1.0,1.0), Vector3d(-1.0,-1.0,-1.0)};
    ROS_WARN("[KRRT]: End pos collide or out of bound, search for other safe end");
    int i = 0;
    for (; i < 20; ++i)
    {
      end_pos += shift[i] * 0.2;
      if (pos_checker_ptr_->validatePosSurround(end_pos))
        break;
    }
    if (i == 20)
    {
      ROS_ERROR("found no valid end pos, plan fail");
      return FAILURE;
    }
  }
  
  if((start_pos - end_pos).norm() < 1e-3 && (start_vel - end_vel).norm() < 1e-4)
  {
    ROS_ERROR("[KRRT]: start state & end state too close");
    return FAILURE;
  }
  
  if (need_consistancy)
  { 
    vis_ptr_->visualizeReplanDire(start_pos, dire, pos_checker_ptr_->getLocalTime());
    replan_normal_ = normal;
    replan_dire_ = dire;
    need_consistancy_ = need_consistancy;
  }
  
  /* construct start and goal nodes */
  start_node_ = start_tree_[1]; //init ptr
  start_node_->x.head(3) = start_pos;
  start_node_->x.segment(3, 3) = start_vel;
  start_node_->x.tail(3) = start_acc;
  start_node_->cost_from_start = 0.0;
  start_node_->tau_from_start = 0.0;
  goal_node_ = start_tree_[0]; //init ptr
  goal_node_->x.head(3) = end_pos;
  if (end_vel.norm() >= vel_limit_)
  {
    end_vel.normalize();
    end_vel = end_vel * vel_limit_;
  }
  goal_node_->x.segment(3, 3) = end_vel;
  goal_node_->x.tail(3) = end_acc;
  goal_node_->cost_from_start = DBL_MAX; //important
  goal_node_->tau_from_start = DBL_MAX; //important
  valid_start_tree_node_nums_ = 2; //start and goal already in start_tree_
  vis_ptr_->visualizeStartAndGoal(start_node_->x, goal_node_->x, pos_checker_ptr_->getLocalTime());

  /* init sampling space */
  vector<pair<Vector3d, Vector3d>> traversal_lines;
	ROS_INFO_STREAM("Kino RRT* starts planning");
  if (bvp_.solve(start_node_->x, goal_node_->x))
  {
    CoefficientMat coeff;
    bvp_.getCoeff(coeff);
    double best_tau = bvp_.getTauStar();
    double best_cost = bvp_.getCostStar();
    ROS_INFO("[KRRT]: Best cost: %lf, best tau: %lf", best_cost, best_tau);
    Piece poly_seg(best_tau, coeff);
    /* for traj vis */
    vector<StatePVA> vis_x;
    vis_x.clear();
    poly_seg.sampleOneSeg(&vis_x);
    vis_ptr_->visualizeStates(vis_x, BestTraj, pos_checker_ptr_->getLocalTime());

    bool vel_cons = poly_seg.checkMaxVelRate(vel_limit_);
    bool acc_cons = poly_seg.checkMaxAccRate(acc_limit_);
    bool pos_cons = getTraversalLines(poly_seg, traversal_lines);
    if (vel_cons && acc_cons && pos_cons)
    {
      ROS_WARN("Best traj collision free, one shot connected");
      goal_node_->cost_from_start = best_cost;
      goal_node_->parent = start_node_;
      goal_node_->tau_from_start = best_tau;
      goal_node_->poly_seg = poly_seg;
      fillTraj(goal_node_, traj_);
      double final_traj_use_time = (ros::Time::now() - plan_start_time).toSec();
  
      /* for traj vis */
      vis_x.clear();
      traj_.sampleWholeTrajectory(&vis_x);
      vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());
      double final_traj_len(0.0), final_traj_duration(0.0), final_traj_acc_itg(0.0), final_traj_jerk_itg(0.0);
      int final_traj_seg_nums(0);
      evaluateTraj(traj_, final_traj_duration, final_traj_len, final_traj_seg_nums, final_traj_acc_itg, final_traj_jerk_itg);
      ROS_INFO_STREAM("[KRRT]: [front-end final path]: " << endl 
          << "    -   seg nums: " << final_traj_seg_nums << endl 
          << "    -   time: " << final_traj_use_time << endl 
          << "    -   acc integral: " << final_traj_acc_itg << endl
          << "    -   jerk integral: " << final_traj_jerk_itg << endl
          << "    -   traj duration: " << final_traj_duration << endl 
          << "    -   path length: " << final_traj_len);
      return SUCCESS;
    }
  }
  else
  {
    ROS_WARN("sth. wrong with the bvp solver");
  }
  sampler_.topoSetup(traversal_lines, start_pos, end_pos);
  vector<Vector3d> p_head, tracks;
  sampler_.getTopo(p_head, tracks);
  vis_ptr_->visualizeTopo(p_head, tracks, pos_checker_ptr_->getLocalTime());

  return rrtStar(start_node_->x, goal_node_->x, tree_node_nums_, search_time, radius_cost_between_two_states_, rewire_);
}

int KRRTPlanner::rrtStar(const StatePVA& x_init, const StatePVA& x_final, int n, double search_time, double radius, const bool rewire)
{ 
  ros::Time rrt_start_time = ros::Time::now();
  ros::Time first_goal_found_time, final_goal_found_time;
  double first_general_cost(0.0);
  
  /* local variables */
  int valid_sample_nums = 0; //random samples in obs free area
  int valid_orphan_nums = 0;
  list<int> orphan_idx_list;
  vector<StatePVA> vis_x;
  bool first_time_find_goal = true;
  bool close_goal_found = false;
  double close_dist = DBL_MAX;
  bool goal_found = false;

  /* kd tree init */
  kdtree *kd_tree = kd_create(3);
  //Add start and goal nodes to kd tree
  kd_insert3(kd_tree, start_node_->x[0], start_node_->x[1], start_node_->x[2], start_node_);
  kd_insert3(kd_tree, goal_node_->x[0], goal_node_->x[1], goal_node_->x[2], goal_node_);

  //TODO changable radius
  double tau_for_instance = radius * 0.75; //maximum
  double fwd_radius_p = getForwardRadius(tau_for_instance, radius);  
  double bcwd_radius_p = getBackwardRadius(tau_for_instance, radius);
  ROS_INFO_STREAM("bcwd_radius_p: " << bcwd_radius_p);
  ROS_INFO_STREAM("fwd_radius_p: " << fwd_radius_p);

  /* main loop */
  // vector<StatePVA> samples, valid_samples;
  int idx = 0;
  for (idx = 0; (ros::Time::now() - rrt_start_time).toSec() < search_time && valid_start_tree_node_nums_ < n ; ++idx) 
  {
    /* biased random sampling */
    StatePVA x_rand;
    bool good_sample = sampler_.samplingOnce(idx, x_rand);
    // samples.push_back(x_rand);
    if (!good_sample) 
    {
      continue;
    }
    // valid_samples.push_back(x_rand);
    ++valid_sample_nums;
    
    /* kd_tree bounds search for parent */
    struct kdres *p_bcwd_nbr_set;
    p_bcwd_nbr_set = getBackwardNeighbour(x_rand, kd_tree, radius - tau_for_instance, bcwd_radius_p);
    if (p_bcwd_nbr_set == nullptr)
    {
      ROS_ERROR("bkwd kd range query error");
      break;
    }
    /* choose parent from kd tree range query result*/
    double min_dist = DBL_MAX;
    double tau_from_s = DBL_MAX;
    RRTNode* x_near = nullptr; //parent
    Piece find_parent_seg;
    double find_parent_tau;
    int bound_node_cnt = 0;
    while(!kd_res_end(p_bcwd_nbr_set)) 
    {
      bound_node_cnt++;
      RRTNodePtr curr_node = (RRTNodePtr)kd_res_item_data(p_bcwd_nbr_set);
      if (curr_node == goal_node_) 
      {
        // goal node can not be parent of any other node
        kd_res_next( p_bcwd_nbr_set );
        continue;
      }
      if (bvp_.solve(curr_node->x, x_rand))
      {
        CoefficientMat coeff;
        bvp_.getCoeff(coeff);
        Piece seg_from_parent = Piece(bvp_.getTauStar(), coeff);
        bool connected = checkSegmentConstraints(seg_from_parent);
        if (connected && min_dist > (curr_node->cost_from_start + bvp_.getCostStar())) 
        {
          min_dist = curr_node->cost_from_start + bvp_.getCostStar();
          tau_from_s = curr_node->tau_from_start + bvp_.getTauStar();
          find_parent_seg = seg_from_parent;
          find_parent_tau = bvp_.getTauStar();
          x_near = curr_node;
          // ROS_INFO("one parent found");
        }
      }
      else
      {
        ROS_ERROR("sth. wrong with the bvp solver");
      }
      kd_res_next(p_bcwd_nbr_set); //go to next in kd tree range query result
    }
    kd_res_free(p_bcwd_nbr_set); //reset kd tree range query

    if (x_near == nullptr) 
    {
      // ROS_INFO_STREAM("no parent for state: " << x_rand.head(6).transpose());
      //no valid parent found, sample next
      if (!allow_orphan_)
        continue;
      else if (valid_orphan_nums < orphan_nums_)
      {
        orphan_idx_list.push_back(valid_orphan_nums);
        valid_orphan_nums++;
        orphans_.push_back(x_rand);
        // vector<StatePVA> vis_orphan;
        // for (auto it = orphan_idx_list.cbegin(); it != orphan_idx_list.cend(); ++it) 
        // {
        //   vis_orphan.push_back(orphans_[*it]);
        // }
        // vis_ptr_->visualizeOrphans(vis_orphan, pos_checker_ptr_->getLocalTime());
        // TODO stock orphans in kd-tree?
      }
    } 
    else 
    {
      /* parent found within radius, then: 
       * 1.add a node to rrt and kd_tree; 
       * 2.rewire. */

      //sample rejection
      x_rand.segment(6, 3) = find_parent_seg.getAcc(find_parent_tau);
      if(bvp_.solve(x_rand, goal_node_->x, ACC_KNOWN))
      {
        if (min_dist + bvp_.getCostStar() >= goal_node_->cost_from_start) 
        {
          // ROS_WARN("parent found but sample rejected");
          continue;
        }
      }
      else
      {
        ROS_ERROR("sth. wrong with the bvp solver");
      }

      /* 1.1 add the randomly sampled node to rrt_tree */
      // ROS_INFO("add the randomly sampled node to rrt_tree");
      RRTNode* sampled_node = start_tree_[valid_start_tree_node_nums_++]; 
      sampled_node->x = x_rand;
      sampled_node->parent = x_near;
      sampled_node->cost_from_start = min_dist;
      sampled_node->tau_from_start = tau_from_s;
      sampled_node->poly_seg = find_parent_seg;
      x_near->children.push_back(sampled_node);
          
      /* 1.2 add the randomly sampled node to kd_tree */
      kd_insert3(kd_tree, x_rand[0], x_rand[1], x_rand[2], sampled_node);
      
      /* try to connect to goal after a valid tree node found */ 
      CoefficientMat coeff;
      bvp_.getCoeff(coeff);
      Piece seg2goal = Piece(bvp_.getTauStar(), coeff);
      bool connected_to_goal = checkSegmentConstraints(seg2goal);
      if (connected_to_goal) 
      {
        /*-- if commented, multiple nodes can connect to goal, but goal still has only one parent --*/
        if (goal_node_->parent) 
        {
          goal_node_->parent->children.remove(goal_node_);
        }
        /*-- --*/
        goal_node_->parent = sampled_node;
        goal_node_->cost_from_start = min_dist + bvp_.getCostStar();
        goal_node_->tau_from_start = tau_from_s + bvp_.getTauStar();
        CoefficientMat goal_coeff;
        bvp_.getCoeff(goal_coeff);
        goal_node_->poly_seg = Piece(bvp_.getTauStar(), goal_coeff);
        sampled_node->children.push_back(goal_node_);
        goal_found = true;
        // ROS_WARN_STREAM("[KRRT]: goal found at " << (ros::Time::now() - rrt_start_time).toSec() << " s , cost: " << goal_node_->cost_from_start << ", tau: " << goal_node_->tau_from_start);
        if (first_time_find_goal) 
        {
          first_general_cost = goal_node_->cost_from_start;
          first_goal_found_time = ros::Time::now();
          first_time_find_goal = false;
          std::vector<double> durs;
          std::vector<CoefficientMat> coeffMats;
          RRTNodePtr node = goal_node_;
          while (node->parent) 
          {
            //new a RRTNodePtr instead of use ptr in start_tree_ because as sampling goes on and rewire happens, 
            //parent, children and other attributes of start_tree_[i] may change.
            durs.push_back(node->poly_seg.getDuration());
            coeffMats.push_back(node->poly_seg.getCoeffMat());
            node = node->parent;
          }
          std::reverse(std::begin(durs), std::end(durs));
          std::reverse(std::begin(coeffMats), std::end(coeffMats));
          first_traj_ = Trajectory(durs, coeffMats);
        }
        if (stop_after_first_traj_found_) break; //stop searching after first time find the goal?
      }
      // else if(allow_close_goal_ && !goal_found)
      // {
      //   double dist_to_goal = dist(x_rand, goal_node_->x);
      //   if (dist_to_goal < close_dist && tau_from_s >= 2)
      //   {
      //     close_dist = dist_to_goal;
      //     ROS_INFO_STREAM("[KRRT]: close goal found, Euc dist to goal: " << dist_to_goal);
      //     close_goal_found = true;
      //     close_goal_node_ = sampled_node;
      //   }
      // }
      /* end of try to connect to goal */
      
      
      /* 2.rewire */
      if (rewire) 
      {
        //kd_tree bounds search
        struct kdres *p_fwd_nbr_set;
        p_fwd_nbr_set = getForwardNeighbour(x_rand, kd_tree, tau_for_instance, fwd_radius_p);
        if (p_fwd_nbr_set == nullptr)
        {
          ROS_ERROR("fwd kd range query error");
          break;
        }
        while(!kd_res_end(p_fwd_nbr_set)) 
        {
          RRTNode* curr_node = (RRTNode*)kd_res_item_data(p_fwd_nbr_set);
          if (curr_node == goal_node_ || curr_node == start_node_) 
          {
            // already tried to connect to goal from random sampled node
            kd_res_next(p_fwd_nbr_set);
            continue;
          }
          Piece seg_rewire;
          if(bvp_.solve(x_rand, curr_node->x, ACC_KNOWN))
          {
            CoefficientMat coeff;
            bvp_.getCoeff(coeff);
            seg_rewire = Piece(bvp_.getTauStar(), coeff);
          }
          else
          {
            ROS_ERROR("sth. wrong with the bvp solver");
          }
          bool connected = checkSegmentConstraints(seg_rewire);
          if (connected && sampled_node->cost_from_start + bvp_.getCostStar() < curr_node->cost_from_start) 
          {
            // If we can get to a node via the sampled_node faster than via it's existing parent then change the parent
            curr_node->parent->children.remove(curr_node);  //DON'T FORGET THIS, remove it form its parent's children list
            curr_node->parent = sampled_node;
            curr_node->cost_from_start = sampled_node->cost_from_start + bvp_.getCostStar();
            curr_node->tau_from_start = sampled_node->tau_from_start + bvp_.getTauStar();
            CoefficientMat coeff;
            bvp_.getCoeff(coeff);
            curr_node->poly_seg = Piece(bvp_.getTauStar(), coeff);
            sampled_node->children.push_back(curr_node);
          }
          /* go to the next entry */
          kd_res_next(p_fwd_nbr_set);
        }
        kd_res_free(p_fwd_nbr_set);
      }/* end of rewire */
      
      /* check orphans */
      if (allow_orphan_)
      {
        vector<int> adopted_orphan_idx;
        for (auto it = orphan_idx_list.cbegin(); it != orphan_idx_list.cend(); ++it) 
        {
          Piece seg_orphan;
          if(bvp_.solve(x_rand, orphans_[*it]))
          {
            CoefficientMat coeff;
            bvp_.getCoeff(coeff);
            seg_orphan = Piece(bvp_.getTauStar(), coeff);
          }
          else
          {
            ROS_ERROR("sth. wrong with the bvp solver");
          }
          bool connected_to_orphan = checkSegmentConstraints(seg_orphan);
          if (connected_to_orphan) 
          {
            /* 1. add the orphan node to rrt_tree */
            RRTNode* orphan_node = start_tree_[valid_start_tree_node_nums_++]; 
            orphan_node->x = orphans_[*it];
            orphan_node->parent = sampled_node;
            orphan_node->cost_from_start = sampled_node->cost_from_start + bvp_.getCostStar();
            orphan_node->tau_from_start = sampled_node->tau_from_start + bvp_.getTauStar();
            CoefficientMat coeff;
            bvp_.getCoeff(coeff);
            orphan_node->poly_seg = Piece(bvp_.getTauStar(), coeff);
            orphan_node->x.segment(6, 3) = orphan_node->poly_seg.getAcc(bvp_.getTauStar());
            sampled_node->children.push_back(orphan_node);
                
            /* 2. add the orphan node to kd_tree */
            kd_insert3(kd_tree, orphan_node->x[0], orphan_node->x[1], orphan_node->x[2], orphan_node);
            
            adopted_orphan_idx.push_back(*it);
            // ROS_WARN("orphan!");
            // break;
          }
        }
        /* 3. remove orphan list */
        for (int i : adopted_orphan_idx)
        {
          orphan_idx_list.remove(i);
        }
      }
    }/* end of find parent */
  }/* end of sample once */

  vis_x.clear();
  vector<Vector3d> knots;
  sampleWholeTree(start_node_, &vis_x, knots);
  vis_ptr_->visualizeStates(vis_x, TreeTraj, pos_checker_ptr_->getLocalTime());
  vis_ptr_->visualizeKnots(knots, pos_checker_ptr_->getLocalTime());
  // vis_ptr_->visualizeSampledState(samples, pos_checker_ptr_->getLocalTime());
  // vis_ptr_->visualizeValidSampledState(valid_samples, pos_checker_ptr_->getLocalTime());

  if (goal_found) 
  {
    final_goal_found_time = ros::Time::now();
    fillTraj(goal_node_, traj_);  
    /* for traj vis */
    vis_x.clear();
    first_traj_.sampleWholeTrajectory(&vis_x);
    vis_ptr_->visualizeStates(vis_x, FirstTraj, pos_checker_ptr_->getLocalTime());
    vis_x.clear();
    traj_.sampleWholeTrajectory(&vis_x);
    vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());
    
    double first_traj_len(0.0), first_traj_duration(0.0), first_traj_acc_itg(0.0), first_traj_jerk_itg(0.0);
    double final_traj_len(0.0), final_traj_duration(0.0), final_traj_acc_itg(0.0), final_traj_jerk_itg(0.0);
    int first_traj_seg_nums(0), final_traj_seg_nums(0);
    evaluateTraj(first_traj_, first_traj_duration, first_traj_len, first_traj_seg_nums, first_traj_acc_itg, first_traj_jerk_itg);
    evaluateTraj(traj_, final_traj_duration, final_traj_len, final_traj_seg_nums, final_traj_acc_itg, final_traj_jerk_itg);
    double first_traj_use_time = first_goal_found_time.toSec()-rrt_start_time.toSec();
    double final_traj_use_time = final_goal_found_time.toSec()-rrt_start_time.toSec();
    ROS_INFO_STREAM("[KRRT]: total sample times: " << idx);
    ROS_INFO_STREAM("[KRRT]: valid sample times: " << valid_sample_nums);
    ROS_INFO_STREAM("[KRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
    ROS_INFO_STREAM("[KRRT]: [front-end first path]: " << endl 
         << "    -   seg nums: " << first_traj_seg_nums << endl 
         << "    -   time: " << first_traj_use_time << " s" << endl 
         << "    -   acc integral: " << first_traj_acc_itg << endl
         << "    -   jerk integral: " << first_traj_jerk_itg << endl
         << "    -   traj duration: " << first_traj_duration << endl 
         << "    -   path length: " << first_traj_len << " m"<< endl 
         << "    -   general cost: " << first_general_cost);
        
    ROS_INFO_STREAM("[KRRT]: [front-end final path]: " << endl 
         << "    -   seg nums: " << final_traj_seg_nums << endl 
         << "    -   time: " << final_traj_use_time << " s"<< endl 
         << "    -   acc integral: " << final_traj_acc_itg << endl
         << "    -   jerk integral: " << final_traj_jerk_itg << endl
         << "    -   traj duration: " << final_traj_duration << endl 
         << "    -   path length: " << final_traj_len << " m"<< endl 
         << "    -   general cost: " << goal_node_->cost_from_start);
    return SUCCESS;
  } 
  // else if (close_goal_found)
  // {
  //   ROS_ERROR("Not connectting to goal, but to close_goal");
  //   final_goal_found_time = ros::Time::now();
  //   fillTraj(close_goal_node_, traj_);  
  //   /* for traj vis */
  //   vis_x.clear();
  //   traj_.sampleWholeTrajectory(&vis_x);
  //   vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());
    
  //   double final_traj_len(0.0), final_traj_duration(0.0), final_traj_acc_itg(0.0), final_traj_jerk_itg(0.0);
  //   int final_traj_seg_nums(0);
  //   evaluateTraj(traj_, final_traj_duration, final_traj_len, final_traj_seg_nums, final_traj_acc_itg, final_traj_jerk_itg);
  //   double final_traj_use_time = final_goal_found_time.toSec()-rrt_start_time.toSec();
  //   ROS_INFO_STREAM("[KRRT]: total sample times: " << idx);
  //   ROS_INFO_STREAM("[KRRT]: valid sample times: " << valid_sample_nums);
  //   ROS_INFO_STREAM("[KRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
  //   ROS_INFO_STREAM("[KRRT]: [front-end final path]: " << endl 
  //        << "    -   seg nums: " << final_traj_seg_nums << endl 
  //        << "    -   time: " << final_traj_use_time << endl 
  //        << "    -   acc integral: " << final_traj_acc_itg << endl
  //        << "    -   jerk integral: " << final_traj_jerk_itg << endl
  //        << "    -   traj duration: " << final_traj_duration << endl 
  //        << "    -   path length: " << final_traj_len);
  //   return SUCCESS_CLOSE_GOAL;
  // }
  else if (allow_close_goal_ && valid_start_tree_node_nums_ > 2)
  {
    ROS_WARN("Not connectting to goal, choose a bypass");
    final_goal_found_time = ros::Time::now();
    chooseBypass(close_goal_node_, start_node_);
    fillTraj(close_goal_node_, traj_);  
    /* for traj vis */
    vis_x.clear();
    traj_.sampleWholeTrajectory(&vis_x);
    vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());
    
    double final_traj_len(0.0), final_traj_duration(0.0), final_traj_acc_itg(0.0), final_traj_jerk_itg(0.0);
    int final_traj_seg_nums(0);
    evaluateTraj(traj_, final_traj_duration, final_traj_len, final_traj_seg_nums, final_traj_acc_itg, final_traj_jerk_itg);
    double final_traj_use_time = final_goal_found_time.toSec()-rrt_start_time.toSec();
    ROS_INFO_STREAM("[KRRT]: total sample times: " << idx);
    ROS_INFO_STREAM("[KRRT]: valid sample times: " << valid_sample_nums);
    ROS_INFO_STREAM("[KRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
    ROS_INFO_STREAM("[KRRT]: [front-end final path]: " << endl 
         << "    -   seg nums: " << final_traj_seg_nums << endl 
         << "    -   time: " << final_traj_use_time << " s"<< endl 
         << "    -   acc integral: " << final_traj_acc_itg << endl
         << "    -   jerk integral: " << final_traj_jerk_itg << endl
         << "    -   traj duration: " << final_traj_duration << endl 
         << "    -   path length: " << final_traj_len << " m" << endl 
         << "    -   general cost: " << close_goal_node_->cost_from_start);
    return SUCCESS_CLOSE_GOAL;
  }
  else if (valid_start_tree_node_nums_ == n)
  {
    ROS_ERROR_STREAM("[KRRT]: NOT CONNECTED TO GOAL after " << n << " nodes added to rrt-tree");
    ROS_INFO_STREAM("[KRRT]: total sample times: " << idx);
    ROS_INFO_STREAM("[KRRT]: valid sample times: " << valid_sample_nums);
    ROS_INFO_STREAM("[KRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
    return FAILURE;
  }
  else if ((ros::Time::now() - rrt_start_time).toSec() >= search_time)
  {
    ROS_ERROR_STREAM("[KRRT]: NOT CONNECTED TO GOAL after " << (ros::Time::now() - rrt_start_time).toSec() << " seconds");
    ROS_INFO_STREAM("[KRRT]: total sample times: " << idx);
    ROS_INFO_STREAM("[KRRT]: valid sample times: " << valid_sample_nums);
    ROS_INFO_STREAM("[KRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
    return FAILURE;
  }
  else
  {
    return FAILURE;
  }
  
}

double KRRTPlanner::getForwardRadius(double tau, double cost)
{
  MatrixXd G(3, 3);
  G.setZero();
  double tau_2 = tau * tau;
  double tau_3 = tau_2 * tau;
  double tau_4 = tau_3 * tau;
  double tau_5 = tau_4 * tau;
  G(0, 0) = 720.0 / tau_5;
  G(1, 1) = 192.0 / tau_3;
  G(2, 2) = 9.0 / tau;
  G(0, 1) = G(1, 0) = -360.0 / tau_4;
  G(0, 2) = G(2, 0) = 60.0 / tau_3;
  G(1, 2) = G(2, 1) = -36.0 / tau_2;
  G = G * rho_ / (cost - tau) * 3.0;
  // ROS_INFO_STREAM("G: \n" << G);
  Eigen::EigenSolver<MatrixXd> es(G);
  // cout << "The eigenvalues of G are:" << endl << es.eigenvalues() << endl;
  // cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
  double radius_p(0.0);
  for (int i = 0; i < 3; ++i)
  {
    radius_p = max(radius_p, sqrt(1.0 / es.eigenvalues()[i].real()) * fabs(es.eigenvectors().col(i)[i]));
  }
  return radius_p * 1.732;
}

double KRRTPlanner::getBackwardRadius(double tau, double cost)
{
  MatrixXd G(3, 3);
  G.setZero();
  double tau_2 = tau * tau;
  double tau_3 = tau_2 * tau;
  double tau_4 = tau_3 * tau;
  double tau_5 = tau_4 * tau;
  G(0, 0) = 720.0 / tau_5;
  G(1, 1) = 192.0 / tau_3;
  G(2, 2) = 9.0 / tau;
  G(0, 1) = G(1, 0) = -360.0 / tau_4;
  G(0, 2) = G(2, 0) = 60.0 / tau_3;
  G(1, 2) = G(2, 1) = -36.0 / tau_2;
  G = G * rho_ / (cost - tau) * 3.0;
  // ROS_INFO_STREAM("G: \n" << G);
  Eigen::EigenSolver<MatrixXd> es(G);
  // cout << "The eigenvalues of G are:" << endl << es.eigenvalues() << endl;
  // cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
  double radius_p(0.0);
  for (int i = 0; i < 3; ++i)
  {
    radius_p = max(radius_p, sqrt(1.0 / es.eigenvalues()[i].real()) * fabs(es.eigenvectors().col(i)[i]));
  }
  return radius_p * 1.732;
}

struct kdres *KRRTPlanner::getForwardNeighbour(const StatePVA& x0, struct kdtree *kd_tree, double tau, double radius_p)
{
  double half_tau_square = tau * tau / 2;
  StatePVA x_ba_tau;
  x_ba_tau[0] = x0[0] + x0[3]*tau + x0[6]*half_tau_square;
  x_ba_tau[1] = x0[1] + x0[4]*tau + x0[7]*half_tau_square;
  x_ba_tau[2] = x0[2] + x0[5]*tau + x0[8]*half_tau_square;
  // x_ba_tau[3] = x0[3] + tau*x0[6];
  // x_ba_tau[4] = x0[4] + tau*x0[7];
  // x_ba_tau[5] = x0[5] + tau*x0[8];
  // x_ba_tau[6] = x0[6];
  // x_ba_tau[7] = x0[7];
  // x_ba_tau[8] = x0[8];

  if (debug_vis_)
  {
    vis_ptr_->visualizeReachPos(FORWARD_REACHABLE_POS, x_ba_tau.head(3), 2 * radius_p, pos_checker_ptr_->getLocalTime());
    // vis_ptr_->visualizeReachVel(1, x_ba_tau.head(3) + x_ba_tau.tail(3), 2 * radius_v, pos_checker_ptr_->getLocalTime());
    // getchar();
  }
  return kd_nearest_range3(kd_tree, x_ba_tau[0], x_ba_tau[1], x_ba_tau[2], radius_p);
}

struct kdres *KRRTPlanner::getBackwardNeighbour(const StatePVA& x1, struct kdtree *kd_tree, double tau, double radius_p)
{
  StatePVA expNegATau_x1;
  double half_tau_square = tau * tau / 2;
  expNegATau_x1[0] = x1[0] - x1[3]*tau + x1[6]*half_tau_square;
  expNegATau_x1[1] = x1[1] - x1[4]*tau + x1[7]*half_tau_square;
  expNegATau_x1[2] = x1[2] - x1[5]*tau + x1[8]*half_tau_square;
  // expNegATau_x1[3] = x1[3] - x1[6]*tau;
  // expNegATau_x1[4] = x1[4] - x1[7]*tau;
  // expNegATau_x1[5] = x1[5] - x1[8]*tau;
  // expNegATau_x1[6] = x1[6];
  // expNegATau_x1[7] = x1[7];
  // expNegATau_x1[8] = x1[8];

  if (debug_vis_)
  {
    vis_ptr_->visualizeReachPos(BACKWARD_REACHABLE_POS, expNegATau_x1.head(3), 2 * radius_p, pos_checker_ptr_->getLocalTime());
    // vis_ptr_->visualizeReachVel(1, expNegATau_x1.head(3) + expNegATau_x1.tail(3), 2 * radius_v, pos_checker_ptr_->getLocalTime());
    // getchar();
  }
  return kd_nearest_range3(kd_tree, expNegATau_x1[0], expNegATau_x1[1], expNegATau_x1[2], radius_p);
}

inline double KRRTPlanner::dist(const StatePVA& x0, const StatePVA& x1)
{
  Vector3d p_diff(x0.head(3) - x1.head(3));
  return p_diff.norm();
}

inline void KRRTPlanner::sampleWholeTree(const RRTNodePtr &root, vector< StatePVA >* vis_x, vector<Vector3d>& knots)
{
  if (root == nullptr) 
    return;
  //whatever dfs or bfs
  RRTNode* node = root;
  std::queue<RRTNode*> Q;
  Q.push(node);
  while (!Q.empty()) 
  {
    node = Q.front();
    Q.pop();
    for (const auto& leafptr : node->children) 
    {
      leafptr->poly_seg.sampleOneSeg(vis_x);
      knots.push_back(leafptr->x.head(3));
      Q.push(leafptr);
    }
  }
}

void KRRTPlanner::fillTraj(const RRTNodePtr &goal_leaf, Trajectory& traj)
{
  std::vector<double> durs;
  std::vector<CoefficientMat> coeffMats;
  RRTNodePtr node = goal_leaf;
  while (node->parent) 
  {
    durs.push_back(node->poly_seg.getDuration());
    coeffMats.push_back(node->poly_seg.getCoeffMat());
    node = node->parent;
  }
  std::reverse(std::begin(durs), std::end(durs));
  std::reverse(std::begin(coeffMats), std::end(coeffMats));
  traj = Trajectory(durs, coeffMats);
}

inline bool KRRTPlanner::checkSegmentConstraints(const Piece &seg)
{
  if (!seg.checkMaxVelRate(vel_limit_))
  {
    // ROS_WARN("vel constraints violate!");
    return false;
  }
  if (!seg.checkMaxAccRate(acc_limit_))
  {
    // ROS_WARN("acc constraints violate!");
    return false;
  }
  if (!pos_checker_ptr_->checkPolySeg(seg))
  {
    // ROS_WARN("pos constraints violate!");
    return false;
  }
  return true;
}

inline bool KRRTPlanner::getTraversalLines(const Piece &seg, vector<pair<Vector3d, Vector3d>> &traversal_lines)
{
  bool res = pos_checker_ptr_->checkPolySeg(seg, traversal_lines);
  return res;
}

void KRRTPlanner::evaluateTraj(const Trajectory& traj, double& traj_duration, double& traj_length, int& seg_nums, 
                               double& acc_integral, double& jerk_integral)
{
  traj_length = 0.0;
  traj_duration = 0.0;
  seg_nums = traj.getPieceNum();
  acc_integral = 0.0;
  jerk_integral = 0.0;

  double d_t = 0.03;
  for (int i = 0; i < seg_nums; ++i)
  {
    double tau = traj[i].getDuration();
    traj_duration += tau;
    for (double t = 0.0; t < tau; t += d_t) 
    {
      Eigen::Vector3d vel, acc, jerk;
      vel = traj[i].getVel(t);
      acc = traj[i].getAcc(t);
      jerk = traj[i].getJerk(t);
      traj_length += vel.norm() * d_t;
      acc_integral += acc.dot(acc) * d_t;
      jerk_integral += jerk.dot(jerk) * d_t;
    }
  }
}

void KRRTPlanner::getTraj(Trajectory &traj)
{
  traj = traj_;
}

void KRRTPlanner::chooseBypass(RRTNodePtr &goal_leaf, const RRTNodePtr &tree_start_node)
{
  goal_leaf = start_tree_[valid_start_tree_node_nums_ - 1];
  double max_score(0.0);
  if (need_consistancy_)
  {
    Vector3d p1 = tree_start_node->x.head(3);
    for (const auto &leaf : tree_start_node->children)
    {
      Vector3d p2 = leaf->x.head(3);
      Vector3d dire = (p2 - p1).normalized();
      double score = dire.dot(replan_dire_);
      if (score >= max_score)
      {
        max_score = score;
        goal_leaf = leaf;
      }
    }
  }
  else
  {
    double close_dist(DBL_MAX);
    for (int i = 2; i < valid_start_tree_node_nums_; ++i)
    {
      double dist_to_goal = dist(start_tree_[i]->x, goal_node_->x);
      if (dist_to_goal < close_dist)
      {
        close_dist = dist_to_goal;
        close_goal_node_ = start_tree_[i];
      }
    }
  }
}

} //namespace tgk_planner
