#include "kino_plan/bvp_solver.h"
#include <iostream>

namespace BVPSolver
{

bool IntegratorBVP::solveDouble()
{
  bool result = calTauStarDouble();
  
  double t2 = tau_star_*tau_star_;
  double t3 = t2*tau_star_;

  coeff_(0, 0) = 0.0;
  coeff_(0, 1) = 0.0;
  coeff_(0, 2) = (2.0*(x0_[0]-x1_[0])+tau_star_*(x0_[3]+x1_[3]))/t3;
  coeff_(0, 3) = -(3.0*(x0_[0]-x1_[0])+tau_star_*(2*x0_[3]+x1_[3]))/t2;
  coeff_(0, 4) = x0_[3];
  coeff_(0, 5) = x0_[0];
  coeff_(1, 0) = 0.0;
  coeff_(1, 1) = 0.0;
  coeff_(1, 2) = (2.0*(x0_[1]-x1_[1])+tau_star_*(x0_[4]+x1_[4]))/t3;
  coeff_(1, 3) = -(3.0*(x0_[1]-x1_[1])+tau_star_*(2*x0_[4]+x1_[4]))/t2;
  coeff_(1, 4) = x0_[4];
  coeff_(1, 5) = x0_[1];
  coeff_(2, 0) = 0.0;
  coeff_(2, 1) = 0.0;
  coeff_(2, 2) = (2.0*(x0_[2]-x1_[2])+tau_star_*(x0_[5]+x1_[5]))/t3;
  coeff_(2, 3) = -(3.0*(x0_[2]-x1_[2])+tau_star_*(2*x0_[5]+x1_[5]))/t2;
  coeff_(2, 4) = x0_[5];
  coeff_(2, 5) = x0_[2];

  return result;
}

/* 
minimize Inte(1 + acc^T * rho * acc, 0, tau)
*/
bool IntegratorBVP::calTauStarDouble()
{
  VectorXd p(5);
  p[0] = 1;
  p[1] = 0;
  p[2] = (x0_[3]*x0_[3] + x0_[3]*x1_[3] + x1_[3]*x1_[3] 
        + x0_[4]*x0_[4] + x0_[4]*x1_[4] + x1_[4]*x1_[4] 
        + x0_[5]*x0_[5] + x0_[5]*x1_[5] + x1_[5]*x1_[5]) * (-4.0) * rho_;
  p[3] =(- x0_[0]*x0_[3] - x0_[0]*x1_[3] + x1_[0]*x0_[3] 
        + x1_[0]*x1_[3] - x0_[1]*x0_[4] - x0_[1]*x1_[4] 
        + x1_[1]*x0_[4] + x1_[1]*x1_[4] - x0_[2]*x0_[5] 
        - x0_[2]*x1_[5] + x1_[2]*x0_[5] + x1_[2]*x1_[5]) * 24.0 * rho_;
  p[4] =(- x0_[0]*x0_[0] + 2.0*x0_[0]*x1_[0] - x1_[0]*x1_[0] 
        - x0_[1]*x0_[1] + 2.0*x0_[1]*x1_[1] - x1_[1]*x1_[1] 
        - x0_[2]*x0_[2] + 2.0*x0_[2]*x1_[2] - x1_[2]*x1_[2]) * 36.0 * rho_;
  std::set<double> roots = RootFinder::solvePolynomial(p, DBL_EPSILON, DBL_MAX, 1e-6);
  
  bool result = false;
  double tau = DBL_MAX;
  double cost = DBL_MAX;
  for (const double& root : roots) 
  {
    double t1 = x0_[0] - x1_[0];
    double t2 = x0_[1] - x1_[1];
    double t3 = x0_[2] - x1_[2];
    double t4 = x0_[3] + x1_[3];
    double t5 = x0_[4] + x1_[4];
    double t6 = x0_[5] + x1_[5];
    double t7 = t1*t1 + t2*t2 + t3*t3;
    double t8 = t4*t4 + t5*t5 + t6*t6 - x0_[3]*x1_[3] - x0_[4]*x1_[4] - x0_[5]*x1_[5];
    double t9 = t1*t4 + t2*t5 + t3*t6;
    
    double current = root + rho_*(t7*12/root/root/root + t8*4/root + t9*12/root/root);
    if (current < cost) 
    {
      tau = root;
      cost = current;
      result = true;
    }
  }
  tau_star_ = tau;
  cost_star_ = cost;
  return result;
}

bool IntegratorBVP::solveTriple()
{
  bool result = calTauStarTriple();
  
  double t2 = tau_star_*tau_star_;
  double t3 = tau_star_*t2;
  double t4 = tau_star_*t3;
  double t5 = tau_star_*t4;
  coeff_(0, 0) = -(12*x0_[0] + 6*tau_star_*x0_[3] + t2*x0_[6] - 12*x1_[0] + 6*tau_star_*x1_[3] - t2*x1_[6])/(2*t5); 
  coeff_(1, 0) = -(12*x0_[1] + 6*tau_star_*x0_[4] + t2*x0_[7] - 12*x1_[1] + 6*tau_star_*x1_[4] - t2*x1_[7])/(2*t5); 
  coeff_(2, 0) = -(12*x0_[2] + 6*tau_star_*x0_[5] + t2*x0_[8] - 12*x1_[2] + 6*tau_star_*x1_[5] - t2*x1_[8])/(2*t5);
  coeff_(0, 1) = -(-30*x0_[0] - 16*tau_star_*x0_[3] - 3*t2*x0_[6] + 30*x1_[0] - 14*tau_star_*x1_[3] + 2*t2*x1_[6])/(2*t4);
  coeff_(1, 1) = -(-30*x0_[1] - 16*tau_star_*x0_[4] - 3*t2*x0_[7] + 30*x1_[1] - 14*tau_star_*x1_[4] + 2*t2*x1_[7])/(2*t4);
  coeff_(2, 1) = -(-30*x0_[2] - 16*tau_star_*x0_[5] - 3*t2*x0_[8] + 30*x1_[2] - 14*tau_star_*x1_[5] + 2*t2*x1_[8])/(2*t4); 
  coeff_(0, 2) = -(20*x0_[0] + 12*tau_star_*x0_[3] + 3*t2*x0_[6] - 20*x1_[0] + 8*tau_star_*x1_[3] - t2*x1_[6])/(2*t3); 
  coeff_(1, 2) = -(20*x0_[1] + 12*tau_star_*x0_[4] + 3*t2*x0_[7] - 20*x1_[1] + 8*tau_star_*x1_[4] - t2*x1_[7])/(2*t3); 
  coeff_(2, 2) = -(20*x0_[2] + 12*tau_star_*x0_[5] + 3*t2*x0_[8] - 20*x1_[2] + 8*tau_star_*x1_[5] - t2*x1_[8])/(2*t3); 
  coeff_(0, 3) = x0_[6]/2; 
  coeff_(1, 3) = x0_[7]/2; 
  coeff_(2, 3) = x0_[8]/2;
  coeff_(0, 4) = x0_[3]; 
  coeff_(1, 4) = x0_[4];
  coeff_(2, 4) = x0_[5]; 
  coeff_(0, 5) = x0_[0]; 
  coeff_(1, 5) = x0_[1]; 
  coeff_(2, 5) = x0_[2];

  return result;
}

// /* 
// minimize Inte(1 + acc^T * rho * acc, 0, tau)
// */
// bool IntegratorBVP::calTauStarTriple()
// {
//   VectorXd p(5);
//   p[0] = 35 + rho_*(3*x0_[6]*x0_[6] + 3*x0_[7]*x0_[7] + 3*x0_[8]*x0_[8] 
//                     + x0_[6]*x1_[6] + 3*x1_[6]*x1_[6] + x0_[7]*x1_[7]
//                     + 3*x1_[7]*x1_[7] + x0_[8]*x1_[8] + 3*x1_[8]*x1_[8]);
//   p[1] = 0;
//   p[2] = -6*rho_*(32*x0_[3]*x0_[3] + 32*x0_[4]*x0_[4] + 32*x0_[5]*x0_[5]
//                 + 5*x0_[0]*x0_[6] + 5*x0_[1]*x0_[7] + 5*x0_[2]*x0_[8] 
//                 - 5*x0_[6]*x1_[0] - 5*x0_[7]*x1_[1] - 5*x0_[8]*x1_[2] 
//                 + 36*x0_[3]*x1_[3] + 32*x1_[3]*x1_[3] + 36*x0_[4]*x1_[4] 
//                 + 32*x1_[4]*x1_[4] + 36*x0_[5]*x1_[5] + 32*x1_[5]*x1_[5] 
//                 - 5*x0_[0]*x1_[6] + 5*x1_[0]*x1_[6] - 5*x0_[1]*x1_[7] 
//                 + 5*x1_[1]*x1_[7] - 5*x0_[2]*x1_[8] + 5*x1_[2]*x1_[8]);
//   p[3] = -1200*rho_*(x0_[2]*x0_[5] - x0_[3]*x1_[0] - x0_[4]*x1_[1] - x0_[5]*x1_[2] 
//                   - x1_[0]*x1_[3] + x0_[0]*(x0_[3] + x1_[3]) - x1_[1]*x1_[4] 
//                   + x0_[1]*(x0_[4] + x1_[4]) + x0_[2]*x1_[5] - x1_[2]*x1_[5]);
//   p[4] = -1800*rho_*(x0_[0]*x0_[0] + x0_[1]*x0_[1] + x0_[2]*x0_[2] 
//                 - 2*x0_[0]*x1_[0] + x1_[0]*x1_[0] - 2*x0_[1]*x1_[1] 
//                   + x1_[1]*x1_[1] - 2*x0_[2]*x1_[2] + x1_[2]*x1_[2]);
//   std::set<double> roots = RootFinder::solvePolynomial(p, DBL_EPSILON, DBL_MAX, FLT_EPSILON);
  
//   bool result = false;
//   double tau = DBL_MAX;
//   double cost = DBL_MAX;
  
//   for (const double& root : roots) 
//   {
//     double current = (1/(35*root*root*root))
//     *(600*rho_*((x0_[0] - x1_[0])*(x0_[0] - x1_[0]) 
//               + (x0_[1] - x1_[1])*(x0_[1] - x1_[1]) 
//               + (x0_[2] - x1_[2])*(x0_[2] - x1_[2]))
//     + 600*rho_*root*((x0_[0] - x1_[0])*(x0_[3] + x1_[3]) 
//                       + (x0_[1] - x1_[1])*(x0_[4] + x1_[4]) 
//                       + (x0_[2] - x1_[2])*(x0_[5] + x1_[5]))
//     + 6*rho_*root*root*(32*x0_[3]*x0_[3] + 32*x0_[4]*x0_[4] + 36*x0_[3]*x1_[3] 
//                               + 36*x0_[4]*x1_[4] + 32*(x1_[3]*x1_[3] + x1_[4]*x1_[4]) 
//                               + 4*(8*x0_[5]*x0_[5] + 9*x0_[5]*x1_[5] + 8*x1_[5]*x1_[5])
//                               + 5*(x0_[0] - x1_[0])*(x0_[6] - x1_[6]) 
//                               + 5*(x0_[1] - x1_[1])*(x0_[7] - x1_[7]) 
//                               + 5*(x0_[2] - x1_[2])*(x0_[8] - x1_[8]))
//     + 2*rho_*root*root*root*(11*x0_[3]*x0_[6] + 11*x0_[4]*x0_[7] 
//                                       + 11*x0_[5]*x0_[8] + 4*x0_[6]*x1_[3] 
//                                       + 4*x0_[7]*x1_[4] + 4*x0_[8]*x1_[5] 
//                                       - 4*x0_[3]*x1_[6] - 11*x1_[3]*x1_[6] 
//                                       - 4*x0_[4]*x1_[7] - 11*x1_[4]*x1_[7] 
//                                       - 4*x0_[5]*x1_[8] - 11*x1_[5]*x1_[8])
//     + root*root*root*root*(35 + rho_*(3*x0_[6]*x0_[6] + 3*x0_[7]*x0_[7] 
//                                                       + x0_[6]*x1_[6] + x0_[7]*x1_[7] 
//                                                       + 3*(x0_[8]*x0_[8] + x1_[6]*x1_[6] + x1_[7]*x1_[7])
//                                                       + x0_[8]*x1_[8] + 3*x1_[8]*x1_[8])));
//     if (current < cost) 
//     {
//       tau = root;
//       cost = current;
//       result = true;
//     }
//   }

//   tau_star_ = tau;
//   cost_star_ = cost;
//   return result;
// }

/* 
minimize Inte(1 + jerk^T * rho * jerk, 0, tau)
*/
bool IntegratorBVP::calTauStarTriple()
{
  double t1 = 3*(x0_[6]*x0_[6] + x0_[7]*x0_[7] + x0_[8]*x0_[8] + x1_[6]*x1_[6] + x1_[7]*x1_[7] + x1_[8]*x1_[8]);
  double t2 = t1 - 2*(x0_[6]*x1_[6] + x0_[7]*x1_[7] + x0_[8]*x1_[8]);
  double t3 = 3*(x0_[3]*x0_[6] + x0_[4]*x0_[7] + x0_[5]*x0_[8] - x1_[3]*x1_[6] - x1_[4]*x1_[7] - x1_[5]*x1_[8]);
  double t4 = t3 + 2*(x0_[6]*x1_[3] + x0_[7]*x1_[4] + x0_[8]*x1_[5] - x0_[3]*x1_[6] - x0_[4]*x1_[7] - x0_[5]*x1_[8]);
  double t5 = 8*(x0_[3]*x0_[3] + x0_[4]*x0_[4] + x0_[5]*x0_[5] + x1_[3]*x1_[3] + x1_[4]*x1_[4] + x1_[5]*x1_[5]);
  double t6 = t5 + 5*((x0_[0]-x1_[0])*(x0_[6]-x1_[6]) + (x0_[1]-x1_[1])*(x0_[7]-x1_[7]) + (x0_[2]-x1_[2])*(x0_[8]-x1_[8]));
  double t7 = t6 + 14*(x0_[3]*x1_[3] + x0_[4]*x1_[4] + x0_[5]*x1_[5]);
  double t8 = (x0_[0]-x1_[0])*(x0_[3]+x1_[3]) + (x0_[1]-x1_[1])*(x0_[4]+x1_[4]) + (x0_[2]-x1_[2])*(x0_[5]+x1_[5]);
  double t9 = (x0_[0]-x1_[0])*(x0_[0]-x1_[0]) + (x0_[1]-x1_[1])*(x0_[1]-x1_[1]) + (x0_[2]-x1_[2])*(x0_[2]-x1_[2]);

  VectorXd p(7);
  p[0] = 1.0;
  p[1] = 0.0;
  p[2] = - 3*rho_*t2;
  p[3] = - 48*rho_*t4;
  p[4] = - 72*rho_*t7;
  p[5] = - 2800*rho_*t8;
  p[6] = - 3600*rho_*t9;
  std::set<double> roots = RootFinder::solvePolynomial(p, DBL_EPSILON, DBL_MAX, 1e-6);
  
  bool result = false;
  double tau = DBL_MAX;
  double cost = DBL_MAX;
  
  for (const double& root : roots) 
  {
    double root2 = root*root;
    double root3 = root2*root;
    double root4 = root3*root;
    double root5 = root4*root;
    double root6 = root5*root;
    
    double current = (root6 + rho_*(720*t9 + 720*root*t8 + 24*root2*t7 + 24*root3*t4 + 3*root4*t2)) / root5;

    if (current < cost) 
    {
      tau = root;
      cost = current;
      result = true;
    }
  }

  tau_star_ = tau;
  cost_star_ = cost;
  return result;
}

/* 
minimize Inte(1 + jerk^T * rho * jerk, 0, tau)
*/
bool IntegratorBVP::calTauStarTripleAccUnknown()
{
  Eigen::Vector3d x0_012 = x0_.head(3), x0_345 = x0_.segment(3, 3), x0_678 = x0_.tail(3);
  Eigen::Vector3d x1_012 = x1_.head(3), x1_345 = x1_.segment(3, 3), x1_678 = x1_.tail(3);
  Eigen::Vector3d x0_diff_x1_012 = x0_012 - x1_012, x0_plus_x1_345 = 5*x0_345 + 3*x1_345;
  
  double t1 = rho_*x0_678.dot(x0_678);
  double t2 = rho_*(7*x0_345.dot(x0_678) + 3*x0_678.dot(x1_345));
  double t3 = rho_*(8*x0_345.dot(x0_345) + 3*x1_345.dot(x1_345) + 5*x0_678.dot(x0_diff_x1_012) + 9*x0_345.dot(x1_345));
  double t4 = rho_*x0_diff_x1_012.dot(x0_plus_x1_345);
  double t5 = rho_*x0_diff_x1_012.dot(x0_diff_x1_012);

  VectorXd p(7);
  p[0] = 1.0;
  p[1] = 0.0;
  p[2] = - 8*t1;
  p[3] = - 16*t2;
  p[4] = - 48*t3;
  p[5] = - 320*t4;
  p[6] = - 1600*t5;
  std::set<double> roots = RootFinder::solvePolynomial(p, DBL_EPSILON, DBL_MAX, 1e-6);
  
  bool result = false;
  double tau = DBL_MAX;
  double cost = DBL_MAX;
  
  for (const double& root : roots) 
  {
    double root2 = root*root;
    double root3 = root2*root;
    double root4 = root3*root;
    double root5 = root4*root;
    double root6 = root5*root;
    
    double current = (root6 + 320*t5 + 80*root*t4 + 16*root2*t3 + 8*root3*t2 + 8*root4*t1) / root5;

    if (current < cost) 
    {
      tau = root;
      cost = current;
      result = true;
    }
  }

  tau_star_ = tau;
  cost_star_ = cost;
  return result;
}

bool IntegratorBVP::solveTripleAccUnknown()
{
  bool result = calTauStarTripleAccUnknown();
  
  double t2 = tau_star_*tau_star_;
  double t3 = tau_star_*t2;
  double t4 = tau_star_*t3;
  double t5 = tau_star_*t4;
  coeff_(0, 0) = -(8*x0_[0] + 5*tau_star_*x0_[3] + t2*x0_[6] - 8*x1_[0] + 3*tau_star_*x1_[3])/(3*t5); 
  coeff_(1, 0) = -(8*x0_[1] + 5*tau_star_*x0_[4] + t2*x0_[7] - 8*x1_[1] + 3*tau_star_*x1_[4])/(3*t5); 
  coeff_(2, 0) = -(8*x0_[2] + 5*tau_star_*x0_[5] + t2*x0_[8] - 8*x1_[2] + 3*tau_star_*x1_[5])/(3*t5);
  coeff_(0, 1) = -(-50*x0_[0] - 32*tau_star_*x0_[3] - 7*t2*x0_[6] + 50*x1_[0] - 18*tau_star_*x1_[3])/(6*t4);
  coeff_(1, 1) = -(-50*x0_[1] - 32*tau_star_*x0_[4] - 7*t2*x0_[7] + 50*x1_[1] - 18*tau_star_*x1_[4])/(6*t4);
  coeff_(2, 1) = -(-50*x0_[2] - 32*tau_star_*x0_[5] - 7*t2*x0_[8] + 50*x1_[2] - 18*tau_star_*x1_[5])/(6*t4); 
  coeff_(0, 2) = -(20*x0_[0] + 14*tau_star_*x0_[3] + 4*t2*x0_[6] - 20*x1_[0] + 6*tau_star_*x1_[3])/(3*t3); 
  coeff_(1, 2) = -(20*x0_[1] + 14*tau_star_*x0_[4] + 4*t2*x0_[7] - 20*x1_[1] + 6*tau_star_*x1_[4])/(3*t3); 
  coeff_(2, 2) = -(20*x0_[2] + 14*tau_star_*x0_[5] + 4*t2*x0_[8] - 20*x1_[2] + 6*tau_star_*x1_[5])/(3*t3); 
  coeff_(0, 3) = x0_[6]/2; 
  coeff_(1, 3) = x0_[7]/2; 
  coeff_(2, 3) = x0_[8]/2;
  coeff_(0, 4) = x0_[3]; 
  coeff_(1, 4) = x0_[4];
  coeff_(2, 4) = x0_[5]; 
  coeff_(0, 5) = x0_[0]; 
  coeff_(1, 5) = x0_[1]; 
  coeff_(2, 5) = x0_[2];

  return result;
}

/* 
minimize Inte(1 + jerk^T * rho * jerk, 0, tau)
*/
bool IntegratorBVP::calTauStarTripleVelAccUnknown()
{
  Eigen::Vector3d x0_012 = x0_.head(3), x0_345 = x0_.segment(3, 3), x0_678 = x0_.tail(3);
  Eigen::Vector3d x1_012 = x1_.head(3), x1_345 = x1_.segment(3, 3), x1_678 = x1_.tail(3);
  Eigen::Vector3d x0_diff_x1_012 = x0_012 - x1_012;
  
  double t4 = rho_*x0_678.dot(x0_678);
  double t3 = rho_*x0_345.dot(x0_678);
  double t2 = rho_*(x0_345.dot(x0_345) + x0_012.dot(x1_678) - x0_678.dot(x1_012));
  double t1 = rho_*(x0_012.dot(x0_345) - x0_345.dot(x1_012));
  double t0 = rho_*x0_diff_x1_012.dot(x0_diff_x1_012);

  VectorXd p(7);
  p[0] = 1.0;
  p[1] = 0.0;
  p[2] = - 5*t4;
  p[3] = - 40*t3;
  p[4] = - 60*t2;
  p[5] = - 160*t1;
  p[6] = - 100*t0;
  std::set<double> roots = RootFinder::solvePolynomial(p, DBL_EPSILON, DBL_MAX, 1e-6);
  bool result = false;
  double tau = DBL_MAX;
  double cost = DBL_MAX;
  
  for (const double& root : roots) 
  {
    double root2 = root*root;
    double root3 = root2*root;
    double root4 = root3*root;
    double root5 = root4*root;
    double root6 = root5*root;
    
    double current = (root6 + 20*t0 + 40*root*t1 + 20*root2*t2 + 20*root3*t3 + 5*root4*t4) / root5;
    if (current < cost) 
    {
      tau = root;
      cost = current;
      result = true;
    }
  }

  tau_star_ = tau;
  cost_star_ = cost;
  return result;
}

bool IntegratorBVP::solveTripleVelAccUnknown()
{
  bool result = calTauStarTripleVelAccUnknown();
  
  double t2 = tau_star_*tau_star_;
  double t3 = tau_star_*t2;
  double t4 = tau_star_*t3;
  double t5 = tau_star_*t4;
  coeff_(0, 0) = -(2*x0_[0] + 2*tau_star_*x0_[3] + t2*x0_[6] - 2*x1_[0])/(12*t5); 
  coeff_(1, 0) = -(2*x0_[1] + 2*tau_star_*x0_[4] + t2*x0_[7] - 2*x1_[1])/(12*t5); 
  coeff_(2, 0) = -(2*x0_[2] + 2*tau_star_*x0_[5] + t2*x0_[8] - 2*x1_[2])/(12*t5);
  coeff_(0, 1) = (10*x0_[0] + 10*tau_star_*x0_[3] + 5*t2*x0_[6] - 10*x1_[0])/(12*t4);
  coeff_(1, 1) = (10*x0_[1] + 10*tau_star_*x0_[4] + 5*t2*x0_[7] - 10*x1_[1])/(12*t4);
  coeff_(2, 1) = (10*x0_[2] + 10*tau_star_*x0_[5] + 5*t2*x0_[8] - 10*x1_[2])/(12*t4); 
  coeff_(0, 2) = -(10*x0_[0] + 10*tau_star_*x0_[3] + 5*t2*x0_[6] - 10*x1_[0])/(6*t3); 
  coeff_(1, 2) = -(10*x0_[1] + 10*tau_star_*x0_[4] + 5*t2*x0_[7] - 10*x1_[1])/(6*t3); 
  coeff_(2, 2) = -(10*x0_[2] + 10*tau_star_*x0_[5] + 5*t2*x0_[8] - 10*x1_[2])/(6*t3); 
  coeff_(0, 3) = x0_[6]/2; 
  coeff_(1, 3) = x0_[7]/2; 
  coeff_(2, 3) = x0_[8]/2;
  coeff_(0, 4) = x0_[3]; 
  coeff_(1, 4) = x0_[4];
  coeff_(2, 4) = x0_[5]; 
  coeff_(0, 5) = x0_[0]; 
  coeff_(1, 5) = x0_[1]; 
  coeff_(2, 5) = x0_[2];

  return result;
}

}