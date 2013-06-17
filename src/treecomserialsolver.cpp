/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#include "kdl_codyco/treecomserialsolver.hpp"
#include <iostream>
#include <kdl/frames_io.hpp>

namespace KDL {
namespace CoDyCo {
    
    TreeCOMSerialSolver::TreeCOMSerialSolver(const Tree& tree_in,const TreeSerialization& serialization_in) : TreeSerialSolver(tree_in,serialization_in)
    {
 
        subtree_COM.resize(tree.getNrOfSegments());
        subtree_mass.resize(tree.getNrOfSegments());
            
    }
    
    TreeCOMSerialSolver::~TreeCOMSerialSolver() {
    }
    
    int TreeCOMSerialSolver::JntToCOM(const JntArray& q_in, Vector& p_out) {
        //First we check all the sizes:
        if (q_in.rows() != tree.getNrOfJoints()) {
            return -1;
        }
            
        //Sweep from leafs to root, recursion order in reverse
        for(int l=recursion_order.size()-1; l >= 0; l--) {
            int index = recursion_order[l];

            subtree_COM[index] = index2segment[index]->second.segment.getInertia().getCOG();
            subtree_mass[index] = index2segment[index]->second.segment.getInertia().getMass();
            
            for( int j = 0; j < (int)children[index].size(); j++ ) { 
                int s = children[index][j];
                const Segment & son_segment = index2segment[s]->second.segment;
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
        
        #ifndef NDEBUG
        std::cout << "TreeCOMSerialSolver subtree_COM: " << std::endl;
        for( int i = 0; i != subtree_COM.size(); ++i)
            std::cout << i << " " << subtree_COM[i] << "\n"; 
        #endif 
        p_out = subtree_COM[0];
        return 0;
        
    }
}
}
