/**
 * Copyright  (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/com_loops.hpp"
#include "kdl_codyco/jacobian_loops.hpp"

#include <Eigen/Core>

namespace KDL {
namespace CoDyCo {
	
	template<class Derived>
	inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> & vec)
	{
		EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
		return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
	}
	
	void get6DRigidBodyInertia(const KDL::RigidBodyInertia & kdl_inertia, 
							   Eigen::Matrix<double, 6, 6> & eigen_inertia)
	{
		Eigen::Matrix3d skew_first_moment_of_mass = skew(Eigen::Map<Eigen::Vector3d>((kdl_inertia.getMass()*kdl_inertia.getCOG()).data));
		eigen_inertia << kdl_inertia.getMass()*Eigen::Matrix3d::Identity(),  -skew_first_moment_of_mass,
						 skew_first_moment_of_mass					,  Eigen::Map<Eigen::Matrix3d>(kdl_inertia.getRotationalInertia().data);
		return;
	}
	
	
	void getCenterOfMassLoop(const TreeGraph & tree_graph,
                            const JntArray &q, 
                            const Traversal & traversal,
                            std::vector<KDL::Vector>& subtree_COM,
							std::vector<double>& subtree_mass,
                            Vector & com,
                            int part_id)
    {
	   for(int l=traversal.order.size()-1; l>=0; l-- ) {
            LinkMap::const_iterator link = traversal.order[l];
            
            #ifndef NDEBUG
            //std::cerr << "Traversal size " << traversal.order.size() << std::endl;
            //std::cerr << "TreeCOMSolver: considering link " << link->second.link_name << " " << link->second.link_nr << std::endl;
            #endif
            //if all part is considered, or this link belong to the considered part
            if( part_id < 0 || part_id == (int)link->body_part_nr ) {
                subtree_COM[link->link_nr] = link->I.getCOG();
                subtree_mass[link->link_nr] = link->I.getMass();
            } else {
                subtree_COM[link->link_nr] = Vector::Zero();
                subtree_mass[link->link_nr] = 0.0;
            }
            
            for(int j = 0; j < (int)link->getNrOfAdjacentLinks(); j++ ) {
                LinkMap::const_iterator next_link = link->adjacent_link[j];
                if( next_link != traversal.parent[link->link_nr] ) {
                    int index = link->link_nr;
                    int s = next_link->link_nr;
                    double joint_position;
                    if(link->adjacent_joint[j]->joint.getType() != Joint::None) {
                        joint_position = q(link->adjacent_joint[j]->q_nr);
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
	
	void getMomentumJacobianLoop(const TreeGraph & tree_graph,
                                 const JntArray &q, 
                                 const Traversal & traversal,
		                         const std::vector<Frame>& X_b,
                                 Jacobian & jacobian_momentum,
                                 Jacobian & buffer_1,
                                 Jacobian & buffer_2,
                                 RigidBodyInertia total_inertia,
                                 int part_id)
    {
		Eigen::Matrix<double, 6, 6> eigen_inertia; //The spatial inertia matrix 
		KDL::RigidBodyInertia kdl_inertia;
		
		SetToZero(jacobian_momentum);
		
		total_inertia = RigidBodyInertia::Zero();
		
		for(int l=traversal.order.size()-1; l>=0; l-- ) {
            
            LinkMap::const_iterator link = traversal.order[l];
            			
            //if all part is considered, or this link belong to the considered part
            if( part_id < 0 || part_id == (int)link->body_part_nr ) {
				//\todo improve this code, that is like o(n^3)
				//Get the floating base jacobian for current link (expressed in local frame)
				getFloatingBaseJacobianLoop(tree_graph,q,traversal,link->link_nr,buffer_1);
				
				//Multiply the jacobian with the 6DOF inertia
				get6DRigidBodyInertia(link->getInertia(),eigen_inertia);
				buffer_2.data = eigen_inertia*buffer_1.data;
				
				//Project the jacobian to the base frame
				changeRefFrame(buffer_2, X_b[link->link_nr],buffer_1);
				
				//Add the compute jacobian to the total one
				jacobian_momentum.data += buffer_1.data;
				
				//Calculate also the total inertia, by project each inertia in the base reference frame
				total_inertia = total_inertia + X_b[link->link_nr]*link->getInertia();
            }
        }
        return;
	}
	
}
}
