/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef KDL_CODYCO_TREE_INERTIALPARAMETERS_HPP
#define KDL_CODYCO_TREE_INERTIALPARAMETERS_HPP

#ifdef __DEPRECATED
  #warning <treeinertialparameters.hpp> is deprecated.
#endif

#include "undirectedtree.hpp"
#include "undirectedtreesolver.hpp"
#include "regressor_utils.hpp"
#include "treeserialization.hpp"

#include <Eigen/Core>

#include <kdl/rotationalinertia.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>


namespace KDL {
namespace CoDyCo {



    /**
     * \brief class encapsulating methods relative to inertial parameters
     * \todo What if Chain changes?
     */
     class TreeInertialParametersRegressor : public UndirectedTreeSolver
     {
         private:
            void updateParams();
            bool changeInertialParametersRecursive(const Eigen::VectorXd & new_chain_param, Tree & new_tree, SegmentMap::const_iterator root, const std::string& hook_name) ;

            int nrOfLinks;

            std::string root_name;

            //Frame of link i with respect to the base
            std::vector<Frame> X_b;


            std::vector<Twist> v;
            std::vector<Twist> a;

            Tree tree;

            Twist ag;


            Eigen::VectorXd tree_param;

            //Indicator function
            //std::vector< std::vector<bool> > indicator_function; /**< Indicator function: indicator_function(i,j) == true iff link i has en effect on dynamics of joint j*/

        public:
            /**
             * Constructor, it will allocate all the necessary memory.
             *
             * @param tree the used tree, a reference to this tree is stored.
             */
            TreeInertialParametersRegressor(Tree& tree, Vector grav=Vector::Zero(),const TreeSerialization & serialization=TreeSerialization());
            ~TreeInertialParametersRegressor(){};

            Eigen::VectorXd getInertialParameters();

            UndirectedTree getUndirectedTree() { return undirected_tree; };

             /**
             * Get a copy of the current KDL::Tree, with modified inertial parameters.
             *
             * @param new_chain_param the inertial parameters vector
             * @param new_tree a reference to the output chain
             * @return false in case of some error, true otherwise
             */
            bool changeInertialParameters(const Eigen::VectorXd & new_chain_param, Tree& new_tree);


             /**
             * Return the regressor for fixed base dynamics
             *
             * It replicates the TreeIdSolver_RNE::CartToJnt, the only difference
             * it is that it outputs the regressor matrix instead of result.
             *
             * @param dynamics_regressor a (6+nj)x(10*ns) output matrix
             *
             */
            int dynamicsRegressor(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, Eigen::MatrixXd & dynamics_regressor);


            /**
             * Return the regressor for floating base dynamics
             *
             * It replicates the TreeIdSolver_RNE::CartToJnt, the only difference
             * it is that it outputs the regressor matrix instead of result.
             *
             * @param dynamics_regressor a (6+nj)x(10*ns) output matrix
             *
             */
            int dynamicsRegressor(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot,  const Twist& base_velocity, const Twist& base_acceleration, Eigen::MatrixXd & dynamics_regressor);

        };
}
}


#endif
