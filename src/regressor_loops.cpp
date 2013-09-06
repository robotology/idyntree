/**
 * Copyright  (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/regressor_loops.hpp"
#include "kdl_codyco/regressor_utils.hpp"

#ifndef NDEBUG
#include <iostream>
#endif

namespace KDL {
namespace CoDyCo {
    

void dynamicsRegressorLoop(const TreeGraph & tree_graph,
                         const KDL::JntArray &q, 
                         const Traversal & traversal,
                         const std::vector<Frame>& X_b,
                         const std::vector<Twist>& v,
                         const std::vector<Twist>& a,
                        Eigen::MatrixXd & dynamics_regressor)
{
        dynamics_regressor.setZero();
        
        Eigen::Matrix<double, 6, 10> netWrenchRegressor_i;
        
        for(int l =(int)traversal.order.size()-1; l >= 0; l-- ) {
            LinkMap::const_iterator link = traversal.order[l];
            int i = link->getLinkIndex();
     
            //Each link affects the dynamics of the joints from itself to the base
            netWrenchRegressor_i = netWrenchRegressor(v[i],a[i]);
            
            //Base dynamics
            dynamics_regressor.block(0,(int)(10*i),6,10) = WrenchTransformationMatrix(X_b[i])*netWrenchRegressor_i;
            
            //dynamics_regressor.block(0,(int)(10*i),6,10) = WrenchTransformationMatrix(X_b[i])*netWrenchRegressor_i;
    
            LinkMap::const_iterator child_link = link;
            LinkMap::const_iterator parent_link=traversal.parent[link->getLinkIndex()];
            while( child_link != traversal.order[0] ) {
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
                parent_link = traversal.parent[child_link->getLinkIndex()];
            } 
        }
}

void inertialParametersVectorLoop(const TreeGraph & tree_graph,
                                  Eigen::VectorXd & parameters_vector)
{
    for(int i=0; i < (int)tree_graph.getNrOfLinks(); i++ ) {
            parameters_vector.segment(i*10,10) = Vectorize(tree_graph.getLink(i)->getInertia());
    }
}
    
}
}
