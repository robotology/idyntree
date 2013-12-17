/**
 * Copyright  (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/com_loops.hpp"
#include "kdl_codyco/jacobian_loops.hpp"

#include "kdl_codyco/momentumjacobian.hpp"

#include "kdl_codyco/utils.hpp"

#ifndef NDEBUG
#include <iostream>
#endif

#include <Eigen/Core>

namespace KDL {
namespace CoDyCo {

    
    void get6DRigidBodyInertia(const KDL::RigidBodyInertia & kdl_inertia, 
                               Eigen::Matrix<double, 6, 6> & eigen_inertia)
    {
        Eigen::Matrix3d skew_first_moment_of_mass = skew(Eigen::Map<Eigen::Vector3d>((kdl_inertia.getMass()*kdl_inertia.getCOG()).data));
        eigen_inertia << kdl_inertia.getMass()*Eigen::Matrix3d::Identity(),  -skew_first_moment_of_mass,
                         skew_first_moment_of_mass,  Eigen::Map<Eigen::Matrix3d>(kdl_inertia.getRotationalInertia().data);
        return;
    }
    
    
    void getCenterOfMassLoop(const TreeGraph & ,
                            const KDL::JntArray &q, 
                            const Traversal & traversal,
                            std::vector<KDL::Vector>& subtree_COM,
                            std::vector<double>& subtree_mass,
                            Vector & com,
                            int part_id)
    {
        for(int l=traversal.getNrOfVisitedLinks()-1; l>=0; l-- ) {
            LinkMap::const_iterator link = traversal.getOrderedLink(l);
            
            #ifndef NDEBUG
            //std::cerr << "Traversal size " << traversal.order.size() << std::endl;
            //std::cerr << "TreeCOMSolver: considering link " << link->second.link_name << " " << link->second.link_nr << std::endl;
            #endif
            //if all part is considered, or this link belong to the considered part
            if( part_id < 0 || part_id == (int)link->getBodyPartIndex() ) {
                subtree_COM[link->getLinkIndex()] = link->getInertia().getCOG();
                subtree_mass[link->getLinkIndex()] = link->getInertia().getMass();
            } else {
                subtree_COM[link->getLinkIndex()] = Vector::Zero();
                subtree_mass[link->getLinkIndex()] = 0.0;
            }
            
            for(int j = 0; j < (int)link->getNrOfAdjacentLinks(); j++ ) {
                LinkMap::const_iterator next_link = link->getAdjacentLink(j);
                if( next_link != traversal.getParentLink(link) ) {
                    int index = link->getLinkIndex();
                    int s = next_link->getLinkIndex();
                    double joint_position;
                    if(link->getAdjacentJoint(j)->getJoint().getType() != Joint::None) {
                        joint_position = q(link->getAdjacentJoint(j)->getDOFIndex());
                    } else {
                        joint_position = 0;
                    }    
                   
                    
                    /**\todo solve issue: very little values of mass could cause numerical problems */
                    if( subtree_mass[s] > 0.0 ||  subtree_mass[index] > 0.0 ) { 
                        subtree_COM[index] = (subtree_mass[index]*subtree_COM[index] +
                                            subtree_mass[s]*((link->pose(j,joint_position)).Inverse()*subtree_COM[s]))
                                            /
                                            (subtree_mass[index]+subtree_mass[s]);
                        subtree_mass[index] = subtree_mass[index] + subtree_mass[s];
                    }   
                    
                }
            }
        }
        
        com = subtree_COM[0];
    }
    
    void getMomentumJacobianLoop(const UndirectedTree & undirected_tree,
                                 const KDL::JntArray &q, 
                                 const Traversal & traversal,
                                 const std::vector<Frame>& X_b,
                                 MomentumJacobian & jacobian_momentum,
                                 Jacobian & buffer_jac,
                                 MomentumJacobian & buffer_momentum_jac,
                                 RigidBodyInertia & total_inertia,
                                 int part_id)
    {
        Eigen::Matrix<double, 6, 6> eigen_inertia; //The spatial inertia matrix 
        KDL::RigidBodyInertia kdl_inertia;
    
        SetToZero(jacobian_momentum);
    
        total_inertia = RigidBodyInertia::Zero();
        
        for(int l=traversal.getNrOfVisitedLinks()-1; l>=0; l-- ) {
            
            LinkMap::const_iterator link = traversal.getOrderedLink(l);

            //if all part is considered, or this link belong to the considered part
            if( part_id < 0 || part_id == (int)link->getBodyPartIndex() ) {
                //\todo improve this code, that is like o(n^2)
                //It is easy to implement a o(n) version of it
                //Get the floating base jacobian for current link (expressed in local frame)
                getFloatingBaseJacobianLoop(undirected_tree,q,traversal,link->getLinkIndex(),buffer_jac);
                
                //Multiply the jacobian with the 6DOF inertia
                /** \todo add a proper method for doing this operation */
                //get6DRigidBodyInertia(link->getInertia(),eigen_inertia);
                multiplyInertiaJacobian(buffer_jac,link->getInertia(),buffer_momentum_jac);
                
                //Project the jacobian to the base frame
                buffer_momentum_jac.changeRefFrame(X_b[link->getLinkIndex()]);

                //Add the compute jacobian to the total one
                jacobian_momentum.data += buffer_momentum_jac.data;
                
                //Calculate also the total inertia, by project each inertia in the base reference frame
                #ifndef NDEBUG
                //std::cerr << "Total_inertia mass " << total_inertia.getMass() << std::endl;
                #endif
                kdl_inertia = total_inertia + X_b[link->getLinkIndex()]*link->getInertia();
                total_inertia = kdl_inertia;
            }
        }
        return;
    }
    
    void getCOMJacobianLoop(const UndirectedTree & undirected_tree,
                           const KDL::JntArray &q, 
                           const Traversal & traversal,
                           const std::vector<Frame>& X_b,
                           Jacobian & jac,
                           Jacobian & buffer_jac,
                           int part_id)
    {
        
        assert(undirected_tree.getNrOfDOFs()+6 == jac.columns());
        
        SetToZero(jac);
    
        double m = 0;
            
        for(int l=traversal.getNrOfVisitedLinks()-1; l>=0; l-- ) {
            
            LinkMap::const_iterator link = traversal.getOrderedLink(l);

            //if all part is considered, or this link belong to the considered part
            if( part_id < 0 || part_id == (int)link->getBodyPartIndex() ) {
                //\todo improve this code, that is like o(n^2)
                //It is easy to implement a o(n) version of it
                //Get the floating base jacobian for current link (expressed in local frame)
                getFloatingBaseJacobianLoop(undirected_tree,q,traversal,link->getLinkIndex(),buffer_jac);
                
                double m_i = link->getInertia().getMass();
                
                //Change the pole of the jacobian in the link COM
                buffer_jac.changeRefPoint(link->getInertia().getCOG());
                
                //Change the orientation to the one of the base
                buffer_jac.changeBase(X_b[link->getLinkIndex()].M);

                //Add the computed jacobian to the total one, multiplied by the link mass
                jac.data += m_i*buffer_jac.data;
                
                m += m_i;
           
            }
        }
        
        //Divide the jacobian with the normal one
        jac.data /= m;
        
        return;
    }

}
}
