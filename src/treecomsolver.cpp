/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#include "kdl_codyco/treecomsolver.hpp"
#include "kdl_codyco/com_loops.hpp"
#include <iostream>
#include <kdl/frames_io.hpp>

namespace KDL {
namespace CoDyCo {
    
    TreeCOMSolver::TreeCOMSolver(const Tree& tree_in, TreeSerialization serialization_in, TreePartition partition_in): undirected_tree(tree_in,serialization_in,partition_in)
    {
        subtree_COM.resize(undirected_tree.getNrOfLinks());
        subtree_mass.resize(undirected_tree.getNrOfLinks());
        #ifndef NDEBUG
        std::cerr << "Check consistency in the constructor of TreeComSolver" << std::endl;
        //assert(.check_consistency() == 0);
        #endif
        //Using default base
        int ret = undirected_tree.compute_traversal(traversal);
        assert( ret==0 );
        //Avoiding unused variable warning
        ((void)ret);
    }

    TreeCOMSolver::~TreeCOMSolver() {
    }
    
    int TreeCOMSolver::JntToCOM(const KDL::JntArray& q_in, Vector& p_out, const int part_id) {
        //First we check all the sizes:
        if (q_in.rows() != undirected_tree.getNrOfDOFs()) {
            return -1;
        }
        
        #ifndef NDEBUG
        std::cerr << "Check consistency in TreeCOMSolver::JntToCOM" << std::endl;
        //assert(.check_consistency() == 0);
        #endif
        
        /*
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
                        joint_position = q_in(link->adjacent_joint[j]->q_nr);
                    } else {
                        joint_position = 0;
                    }    
                   
                    
                    //\todo solve issue: very little values of mass could cause numerical problems 
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
        
        p_out = subtree_COM[0];
        */
        
        getCenterOfMassLoop(undirected_tree,q_in,traversal,subtree_COM,subtree_mass,p_out,part_id);
        
        return 0;
        
    } 
    
}
}
