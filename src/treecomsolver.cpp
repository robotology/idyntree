/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#include "kdl_codyco/treecomsolver.hpp"
#include <iostream>
#include <kdl/frames_io.hpp>

namespace KDL {
namespace CoDyCo {
    
    TreeCOMSolver::TreeCOMSolver(const Tree& tree_in, TreeSerialization serialization_in): tree_graph(tree_in,serialization_in)
    {
        subtree_COM.resize(tree_graph.getNrOfLinks());
        subtree_mass.resize(tree_graph.getNrOfLinks());
        #ifndef NDEBUG
        std::cerr << "Check consistency in the constructor of TreeComSolver" << std::endl;
        //assert(tree_graph.check_consistency() == 0);
        #endif
        //Using default base
        int ret = tree_graph.compute_traversal(traversal);
        #ifndef NDEBUG
        assert( ret==0 );
        #endif
    }

    TreeCOMSolver::~TreeCOMSolver() {
    }
    
    int TreeCOMSolver::JntToCOM(const JntArray& q_in, Vector& p_out) {
        //First we check all the sizes:
        if (q_in.rows() != tree_graph.getNrOfDOFs()) {
            return -1;
        }
        
        #ifndef NDEBUG
        std::cerr << "Check consistency in TreeCOMSolver::JntToCOM" << std::endl;
        //assert(tree_graph.check_consistency() == 0);
        #endif
        
        for(int l=traversal.order.size()-1; l>=0; l-- ) {
            LinkMap::const_iterator link = traversal.order[l];
            
            #ifndef NDEBUG
            //std::cerr << "Traversal size " << traversal.order.size() << std::endl;
            //std::cerr << "TreeCOMSolver: considering link " << link->second.link_name << " " << link->second.link_nr << std::endl;
            #endif
            subtree_COM[link->second.link_nr] = link->second.I.getCOG();
            subtree_mass[link->second.link_nr] = link->second.I.getMass();
            
            for(int j = 0; j < (int)link->second.getNrOfAdjacentLinks(); j++ ) {
                LinkMap::const_iterator next_link = link->second.adjacent_link[j];
                if( next_link != traversal.parent[link->second.link_nr] ) {
                    int index = link->second.link_nr;
                    int s = next_link->second.link_nr;
                    double joint_position;
                    if(link->second.adjacent_joint[j]->second.joint.getType() != Joint::None) {
                        joint_position = q_in(link->second.adjacent_joint[j]->second.q_nr);
                    } else {
                        joint_position = 0;
                    }    
                    
                    #ifndef NDEBUG
                    std::cout << "Frame X_"<< index <<"_"<<s << std::endl;
                    std::cout << (link->second.pose(j,joint_position)).Inverse() << std::endl;
                        std::cout << "Frame X_"<< s<<"_"<<index << std::endl;
                    std::cout << (link->second.pose(j,joint_position)) << std::endl;
                    #endif
                    
                    subtree_COM[index] = (subtree_mass[index]*subtree_COM[index] +
                                         subtree_mass[s]*((link->second.pose(j,joint_position)).Inverse()*subtree_COM[s]))
                                         /
                                         (subtree_mass[index]+subtree_mass[s]);
                    subtree_mass[index] = subtree_mass[index] + subtree_mass[s];
                }
            }
        }
        
        #ifndef NDEBUG
        std::cout << "TreeCOMsolver subtree_COM: " << std::endl;
        for( int i = 0; i != (int)subtree_COM.size(); ++i)
            std::cout << i << " " << subtree_COM[i] << "\n"; 
        #endif 
        
        p_out = subtree_COM[0];
        return 0;
        
    } 
    
}
}
