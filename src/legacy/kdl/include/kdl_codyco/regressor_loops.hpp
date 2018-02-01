/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef KDL_CODYCO_REGRESSOR_LOOPS_HPP
#define KDL_CODYCO_REGRESSOR_LOOPS_HPP

#ifdef __DEPRECATED
  #warning <regressor_loops.hpp> is deprecated.
#endif

#include <Eigen/Core>
#include <Eigen/Dense>

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

#include "undirectedtree.hpp"

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
void dynamicsRegressorLoop(const UndirectedTree & ,
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
void dynamicsRegressorFixedBaseLoop(const UndirectedTree & undirected_tree,
                                    const KDL::JntArray &q,
                                    const Traversal & traversal,
                                    const std::vector<Frame>& X_b,
                                    const std::vector<Twist>& v,
                                    const std::vector<Twist>& a,
                                    Eigen::MatrixXd & dynamics_regressor);


void inertialParametersVectorLoop(const UndirectedTree & undirected_tree,
                                  Eigen::VectorXd & parameters_vector);

void inertialParametersVectorLoopFakeLinks(const UndirectedTree & ,
                                  Eigen::VectorXd & parameters_vector,
                                  std::vector < std::string > fake_links_names);

void inertialParametersVectorToUndirectedTreeLoopFakeLinks(const Eigen::VectorXd & parameters_vector,
                                           UndirectedTree & undirected_tree,
                                           std::vector< std::string > fake_links_names);
}
}



#endif
