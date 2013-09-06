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
#include <kdl_codyco/treegraph.hpp>

namespace KDL {
namespace CoDyCo {

void dynamicsRegressorLoop(const TreeGraph & tree_graph,
                         const KDL::JntArray &q, 
                         const Traversal & traversal,
                         const std::vector<Frame>& X_b,
                         const std::vector<Twist>& v,
                         const std::vector<Twist>& a,
                        Eigen::MatrixXd & dynamics_regressor);

void inertialParametersVectorLoop(const TreeGraph & tree_graph,
                                  Eigen::VectorXd & parameters_vector);

}
}  



#endif 
