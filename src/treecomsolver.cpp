/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/treecomsolver.hpp"

namespace KDL {
    
    TreeCOMSolver::TreeCOMSolver(const Tree& tree_in, TreeSerialization serialization) : tree(tree_in)
    {
        subtree_COM.resize(tree.getNrOfSegments());
        subtree_mass.resize(tree.getNrOfSegments());
            
        if(!serialization.is_consistent(tree)) {
            serialization = TreeSerialization(tree);
        }
        
        serialization.serialize(tree,mu_root,mu,lambda,link2joint,recursion_order,seg_vector);
        
    }
    
    TreeCOMSolver::~TreeCOMSolver() {
    }
    
    int TreeCOMSolver::JntToCOM(const JntArray& q_in, Vector& p_out) {
        //First we check all the sizes:
        if (q_in.rows() != tree.getNrOfJoints()) {
            return -1;
        }
            
        //Sweep from leafs to root, recursion order in reverse
        for(int l=recursion_order.size()-1; l >= 0; l--) {
            int index = recursion_order[l];

            subtree_COM[index] = seg_vector[index]->second.segment.getInertia().getCOG();
            subtree_mass[index] = seg_vector[index]->second.segment.getInertia().getMass();
            
            for( int j = 0; j < mu[index].size(); j++ ) { 
                int s = mu[index][j];
                const Segment & son_segment = seg_vector[s]->second.segment;
                const RigidBodyInertia & son_inertia = son_segment.getInertia();
                double joint_position;
                
                if(son_segment.getJoint().getType() !=Joint::None){
                    joint_position = q_in(link2joint[s]);
                } else {
                    joint_position = 0;
                }
                subtree_COM[index] = (subtree_mass[index]*subtree_COM[index] +
                                         subtree_mass[s]*(son_segment.pose(joint_position)*subtree_COM[s]))
                                         /
                                         (subtree_mass[index]+subtree_mass[s]);
                    subtree_mass[index] = subtree_mass[index]+subtree_mass[s];
            }
        }
        
    
        p_out = subtree_COM[0];
        return 0;
        
    }
    
}
