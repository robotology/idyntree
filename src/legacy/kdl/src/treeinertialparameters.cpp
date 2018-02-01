/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */



#include <Eigen/Core>
//#include <Eigen/Dense>

#include <kdl/rigidbodyinertia.hpp>
#include <kdl/tree.hpp>

#include <kdl_codyco/treeinertialparameters.hpp>
#include <kdl_codyco/regressor_loops.hpp>
#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/position_loops.hpp>

#ifndef NDEBUG
#include <iostream>
#include <kdl/frames_io.hpp>
#endif

namespace KDL {
namespace CoDyCo {

    void TreeInertialParametersRegressor::updateParams()
    {
        tree_param.resize(10*undirected_tree.getNrOfLinks());

        inertialParametersVectorLoop(undirected_tree,tree_param);
    }

    Eigen::VectorXd TreeInertialParametersRegressor::getInertialParameters()
    {
        if( tree_param.rows() != 10*undirected_tree.getNrOfLinks() ) updateParams();
        return tree_param;
    }

    bool TreeInertialParametersRegressor::changeInertialParameters(const Eigen::VectorXd & new_chain_param,  Tree& new_tree)
    {
        return false;
        SegmentMap::const_iterator root;
        root = tree.getRootSegment();
        new_tree = Tree(root->first);
        return changeInertialParametersRecursive(new_chain_param,new_tree,root,root->first);
    }

    //code modified from bool Tree::addTreeRecursive(..)

    /*
    bool TreeInertialParametersRegressor::changeInertialParametersRecursive(const Eigen::VectorXd & new_chain_param, Tree & new_tree, SegmentMap::const_iterator root, const std::string& hook_name)
    {

        //Working segment object
        Segment seg;
        //get iterator for root-segment
        SegmentMap::const_iterator child;
        //try to add all of root's children
        for (unsigned int i = 0; i < root->second.children.size(); i++) {
            child = root->second.children[i];
            //Modify segment
            seg = child->second.segment;
            seg.setInertia(deVectorize(tree_param.segment(serial.getLinkId(child->first)*10,10)));

            //Try to add the modified child
            if (new_tree.addSegment(seg, hook_name)) {
                //if child is added, add all the child's children
                if (!(this->changeInertialParametersRecursive(new_chain_param,new_tree,child, child->first)))
                    //if it didn't work, return false
                    return false;
            } else
                //If the child could not be added, return false
                return false;
        }
        return true;
    }*/

    TreeInertialParametersRegressor::TreeInertialParametersRegressor(Tree & _tree,Vector grav,const TreeSerialization & serialization)
    : UndirectedTreeSolver(_tree,serialization)
    , nrOfLinks(undirected_tree.getNrOfLinks())
    , X_b(nrOfLinks)
    , v(nrOfLinks)
    , a(nrOfLinks)
    , tree(_tree)
    {
        //Initializing gravitational acceleration (if any)
        ag=-Twist(grav,Vector::Zero());


        undirected_tree.compute_traversal(traversal);

        //Compiling indicator function;
        /*
        for(unsigned int j=0; j < ns; j++ ) {
            indicator_function[j] = std::vector<bool>(nj,false);
        }

        for(int l =(int)traversal.order.size()-1; l >= 0; l-- ) {
                         LinkMap::const_iterator link = traversal.order[l];
			//Each link affects the dynamics of the joints from itself to the base
			LinkMap::const_iterator child_link = link;
			LinkMap::const_iterator parent_link=traversal.parent[link->getIndex()];
			do{
				if( child_link->getJunction(parent_link)->getNrOfDOFs() == 1 )
					indicator_function[link->getLinkIndex()][child_link->getJunction(parent_link)->getDOFIndex()] = true;
				child_link = parent_link;
				parent_link = traversal.parent[child_link->getIndex()];
			} while( child_link != traversal.order[0] );
		}*/

    }

    int TreeInertialParametersRegressor::dynamicsRegressor( const KDL::JntArray &q,
                                                    const KDL::JntArray &q_dot,
                                                    const KDL::JntArray &q_dotdot,
                                                    Eigen::MatrixXd & dynamics_regressor)
    {
        if(q.rows()!=undirected_tree.getNrOfDOFs() || q_dot.rows()!=undirected_tree.getNrOfDOFs() || q_dotdot.rows()!=undirected_tree.getNrOfDOFs() || dynamics_regressor.cols()!=undirected_tree.getNrOfLinks()*10 || dynamics_regressor.rows()!=(undirected_tree.getNrOfDOFs()))
            return -1;

        rneaKinematicLoop(undirected_tree,q,q_dot,q_dotdot,traversal,Twist::Zero(),ag,v,a);

        //Frame orientation loop
        getFramesLoop(undirected_tree,q,traversal,X_b);

        dynamicsRegressorFixedBaseLoop(undirected_tree,q,traversal,X_b,v,a,dynamics_regressor);

        return 0;
    }

    int TreeInertialParametersRegressor::dynamicsRegressor( const KDL::JntArray &q,
                                                    const KDL::JntArray &q_dot,
                                                    const KDL::JntArray &q_dotdot,
                                                    const Twist& base_velocity,
                                                    const Twist& base_acceleration,
                                                    Eigen::MatrixXd & dynamics_regressor)
    {
        if(q.rows()!=undirected_tree.getNrOfDOFs() || q_dot.rows()!=undirected_tree.getNrOfDOFs() || q_dotdot.rows()!=undirected_tree.getNrOfDOFs() || dynamics_regressor.cols()!=10*undirected_tree.getNrOfLinks() || dynamics_regressor.rows()!=(6+undirected_tree.getNrOfDOFs()))
            return -1;

        //kinematic loop
        rneaKinematicLoop(undirected_tree,q,q_dot,q_dotdot,traversal,base_velocity,base_acceleration,v,a);

        #ifndef NDEBUG
        /*
        for(int i=0; i < v.size(); i++ ) {
            std::cout << "Vel and acc (" << i << " ) " << std::endl;
            std::cout << toEigen(v[i]) << std::endl;
            std::cout << toEigen(a[i]) << std::endl;
        }*/
        #endif

        //Frame orientation loop
        getFramesLoop(undirected_tree,q,traversal,X_b);

        #ifndef NDEBUG
        /*
        for(int i=0; i < v.size(); i++ ) {
            std::cout << "frame (" << i << " ) " << std::endl;
            std::cout << X_b[i] << std::endl;        }*/
        #endif

        dynamicsRegressorLoop(undirected_tree,q,traversal,X_b,v,a,dynamics_regressor);

        /*
        for(i=0;i<ns;i++) {

            netWrenchRegressor_i = netWrenchRegressor(v[i],a[i]);

            dynamics_regressor.block(0,(int)(10*i),6,10) = WrenchTransformationMatrix(X_b[i])*netWrenchRegressor_i;

            Frame X_j_i;
            for(j=0;j<ns;j++) {
                X_j_i = X_b[j].Inverse()*X_b[i];

                if( seg_vector[j]->second.segment.getJoint().getType() != Joint::None ) {
                    if( indicator_function[i][link2joint[j]] ) {
                        dynamics_regressor.block(6+link2joint[j],10*i,1,10) =
                            toEigen(S[j]).transpose()*WrenchTransformationMatrix(X_j_i)*netWrenchRegressor_i;
                    }
                }
            }
        }*/

        return 0;

    }
}
}//namespace

