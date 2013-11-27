/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
 
#ifndef KDL_CODYCO_REGRESSOR_LOOPS_HPP
#define KDL_CODYCO_REGRESSOR_LOOPS_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_codyco/undirectedtree.hpp>

namespace KDL {
namespace CoDyCo {

/**
 * Calculate the dynamics regressor, such that if a is the vector of inertial parameters_vector
 * dynamics_regressor*a == | w   |
 *                         | tau |
 * 
 * Where w is the base wrench and tau is the vector of joint torques
 * 
 */
void dynamicsRegressorLoop(const TreeGraph & ,
                           const KDL::JntArray &q, 
                           const Traversal & traversal,
                           const std::vector<Frame>& X_b,
                           const std::vector<Twist>& v,
                           const std::vector<Twist>& a,
                           Eigen::MatrixXd & dynamics_regressor);

/**
 * Calculate the dynamics regressor for a fixed base robot, such that if a is the vector of inertial parameters_vector
 * dynamics_regressor*a ==  tau 
 * 
 * Where tau is the vector of joint torques
 * 
 */
void dynamicsRegressorFixedBaseLoop(const TreeGraph & ,
                           const KDL::JntArray &q, 
                           const Traversal & traversal,
                           const std::vector<Frame>& X_b,
                           const std::vector<Twist>& v,
                           const std::vector<Twist>& a,
                           Eigen::MatrixXd & dynamics_regressor);


void inertialParametersVectorLoop(const TreeGraph & ,
                                  Eigen::VectorXd & parameters_vector);

void inertialParametersVectorLoopFakeLinks(const TreeGraph & ,
                                  Eigen::VectorXd & parameters_vector,
                                  std::vector < std::string > fake_links_names);
}
}  



#endif 
