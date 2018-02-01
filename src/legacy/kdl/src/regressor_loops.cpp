/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#include <kdl_codyco/regressor_loops.hpp>
#include <kdl_codyco/regressor_utils.hpp>

#ifndef NDEBUG
#include <iostream>
#include <kdl/frames_io.hpp>
#endif

namespace KDL {
namespace CoDyCo {


void dynamicsRegressorLoop(const UndirectedTree & ,
                         const KDL::JntArray &q,
                         const Traversal & traversal,
                         const std::vector<Frame>& X_b,
                         const std::vector<Twist>& v,
                         const std::vector<Twist>& a,
                         Eigen::MatrixXd & dynamics_regressor)
{
        dynamics_regressor.setZero();

        Eigen::Matrix<double, 6, 10> netWrenchRegressor_i;

        // Store the base_world translational transform in world orientation
        KDL::Frame world_base_X_world_world = KDL::Frame(-(X_b[traversal.getBaseLink()->getLinkIndex()].p));

        for(int l =(int)traversal.getNrOfVisitedLinks()-1; l >= 0; l-- ) {
            LinkMap::const_iterator link = traversal.getOrderedLink(l);
            int i = link->getLinkIndex();

            //Each link affects the dynamics of the joints from itself to the base
            netWrenchRegressor_i = netWrenchRegressor(v[i],a[i]);

            //Base dynamics
            // The base dynamics is expressed with the orientation of the world but
            // with respect to the base origin
            dynamics_regressor.block(0,(int)(10*i),6,10) = WrenchTransformationMatrix(world_base_X_world_world*X_b[i])*netWrenchRegressor_i;

            //dynamics_regressor.block(0,(int)(10*i),6,10) = WrenchTransformationMatrix(X_b[i])*netWrenchRegressor_i;

            LinkMap::const_iterator child_link = link;
            LinkMap::const_iterator parent_link=traversal.getParentLink(link);
            while( child_link != traversal.getOrderedLink(0) ) {
                if( child_link->getAdjacentJoint(parent_link)->getNrOfDOFs() == 1 ) {
                    #ifndef NDEBUG
                    //std::cerr << "Calculating regressor columns for link " << link->getName() << " and joint " << child_link->getAdjacentJoint(parent_link)->getName() << std::endl;
                    #endif
                    int dof_index = child_link->getAdjacentJoint(parent_link)->getDOFIndex();
                    int child_index = child_link->getLinkIndex();
                    Frame X_j_i = X_b[child_index].Inverse()*X_b[i];
                    dynamics_regressor.block(6+dof_index,10*i,1,10) =
                            toEigen(parent_link->S(child_link,q(dof_index))).transpose()*WrenchTransformationMatrix(X_j_i)*netWrenchRegressor_i;
                }
                child_link = parent_link;
                #ifndef NDEBUG
                //std::cout << "Getting parent link of link of index " << child_link->getName() << " " << child_link->getLinkIndex() << std::endl;
                //std::cout << "Current base " << traversal.order[0]->getName() << " " << traversal.order[0]->getLinkIndex() << std::endl;
                #endif
                parent_link = traversal.getParentLink(child_link);
            }
        }
}

void dynamicsRegressorFixedBaseLoop(const UndirectedTree & ,
                         const KDL::JntArray &q,
                         const Traversal & traversal,
                         const std::vector<Frame>& X_b,
                         const std::vector<Twist>& v,
                         const std::vector<Twist>& a,
                         Eigen::MatrixXd & dynamics_regressor)
{
        dynamics_regressor.setZero();

        Eigen::Matrix<double, 6, 10> netWrenchRegressor_i;

        for(int l =(int)traversal.getNrOfVisitedLinks()-1; l >= 0; l-- ) {
            LinkMap::const_iterator link = traversal.getOrderedLink(l);
            int i = link->getLinkIndex();

            //Each link affects the dynamics of the joints from itself to the base
            netWrenchRegressor_i = netWrenchRegressor(v[i],a[i]);

             //dynamics_regressor.block(0,(int)(10*i),6,10) = WrenchTransformationMatrix(X_b[i])*netWrenchRegressor_i;

            LinkMap::const_iterator child_link = link;
            LinkMap::const_iterator parent_link=traversal.getParentLink(link);
            while( child_link != traversal.getOrderedLink(0) ) {
                if( child_link->getAdjacentJoint(parent_link)->getNrOfDOFs() == 1 ) {
                    int dof_index = child_link->getAdjacentJoint(parent_link)->getDOFIndex();
                    int child_index = child_link->getLinkIndex();
                    Frame X_j_i = X_b[child_index].Inverse()*X_b[i];
                    dynamics_regressor.block(dof_index,10*i,1,10) =
                            toEigen(parent_link->S(child_link,q(dof_index))).transpose()*WrenchTransformationMatrix(X_j_i)*netWrenchRegressor_i;
                }
                child_link = parent_link;
                parent_link = traversal.getParentLink(child_link);
            }
        }
}

void inertialParametersVectorLoop(const UndirectedTree & undirected_tree,
                                  Eigen::VectorXd & parameters_vector)
{
    for(int i=0; i < (int)undirected_tree.getNrOfLinks(); i++ ) {
            parameters_vector.segment(i*10,10) = Vectorize(undirected_tree.getLink(i)->getInertia());
    }
}

void inertialParametersVectorLoopFakeLinks(const UndirectedTree & undirected_tree,
                                           Eigen::VectorXd & parameters_vector,
                                           std::vector< std::string > fake_links_names)
{
    int real_index_loop = 0;
    for(int i=0; i < (int)undirected_tree.getNrOfLinks(); i++ ) {
        if( std::find(fake_links_names.begin(), fake_links_names.end(), undirected_tree.getLink(i)->getName()) == fake_links_names.end() ) {
            parameters_vector.segment(real_index_loop*10,10) = Vectorize(undirected_tree.getLink(i)->getInertia());
            real_index_loop++;
        }
    }
}

void inertialParametersVectorToUndirectedTreeLoop(const Eigen::VectorXd & parameters_vector,
                                                  UndirectedTree & undirected_tree)
{
    for(int i=0; i < (int)undirected_tree.getNrOfLinks(); i++ ) {
            undirected_tree.getLink(i,false)->setInertia(deVectorize(parameters_vector.segment(i*10,10)));
    }
}

void inertialParametersVectorToUndirectedTreeLoopFakeLinks(
                                           const Eigen::VectorXd & parameters_vector,
                                           UndirectedTree & undirected_tree,
                                           std::vector< std::string > fake_links_names)
{
    int real_index_loop = 0;
    for(int i=0; i < (int)undirected_tree.getNrOfLinks(); i++ ) {
        if( std::find(fake_links_names.begin(), fake_links_names.end(), undirected_tree.getLink(i)->getName()) == fake_links_names.end() ) {
            undirected_tree.getLink(i,false)->setInertia(deVectorize(parameters_vector.segment(real_index_loop*10,10)));
            real_index_loop++;
        }
    }
}


}
}
