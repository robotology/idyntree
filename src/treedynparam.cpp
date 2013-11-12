// Copyright  (C)  2009  Dominick Vanthienen <dominick dot vanthienen at mech dot kuleuven dot be>

// Version: 1.0
// Author: Dominick Vanthienen <dominick dot vanthienen at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include <kdl_codyco/treedynparam.hpp>
//#include "frames_io.hpp"
//#include <iostream>

#include <kdl_codyco/regressor_utils.hpp>
#include <Eigen/Dense>
#include <kdl_codyco/crba_loops.hpp>

#ifndef NDEBUG
#include <iostream>
#endif

namespace KDL {
namespace CoDyCo {
    
    TreeDynParam::TreeDynParam(const Tree& _tree, Vector _grav, const TreeSerialization & _serialization):
        UndirectedTreeSolver(_tree,_serialization),
        nj(_tree.getNrOfJoints()),
        ns(_tree.getNrOfSegments()),
        grav(_grav),
        jntarraynull(nj),
        treeidsolver_coriolis( _tree, Vector::Zero()),
        treeidsolver_gravity( _tree, grav),
        wrenchnull(ns,Wrench::Zero()),
        Ic(ns)
    {
        ag=-Twist(grav,Vector::Zero());
        
        #ifndef NDEBUG
        std::cout << "Allocate TreeDynParam, used undirected_tree: " << std::endl;
        std::cout << undirected_tree.toString() << std::endl;
        assert(undirected_tree.check_consistency() == 0);
        assert(undirected_tree.check_consistency(traversal) == 0);
        #endif
    }

    //calculate inertia matrix H
    int TreeDynParam::JntToMass(const JntArray &q, JntSpaceInertiaMatrix& H)
    {
        //Check sizes when in debug mode
        if(q.rows()!=nj || H.rows()!=nj || H.columns()!=nj )
            return -1;
        
        SetToZero(H);
        
        return crba_fixed_base_loop(undirected_tree,traversal,q,Ic,H);
    }

    //calculate coriolis torques C
    int TreeDynParam::JntToCoriolis(const JntArray &q, const JntArray &q_dot, JntArray &coriolis)
    {
        //make a null matrix with the size of q_dotdot and a null wrench
        SetToZero(jntarraynull);

        //the calculation of coriolis matrix C
        return treeidsolver_coriolis.CartToJnt(q, q_dot, jntarraynull, wrenchnull, coriolis);
        
    }

    //calculate gravity torques G
    int TreeDynParam::JntToGravity(const JntArray &q,JntArray &gravity)
    {
        //make a null matrix with the size of q_dotdot and a null wrench
        SetToZero(jntarraynull);
        //the calculation of coriolis matrix C
        return treeidsolver_gravity.CartToJnt(q, jntarraynull, jntarraynull, wrenchnull, gravity);
    }
    
    //calculate coriolis torques 
    int TreeDynParam::JntToCoriolis(const JntArray &q, const JntArray &q_dot, const Twist & base_vel,  JntArray &coriolis_torques, Wrench & coriolis_base_wrench)
    {
        //make a null matrix with the size of q_dotdot and a null wrench
        SetToZero(jntarraynull);

        //the calculation of coriolis matrix C
        return treeidsolver_coriolis.CartToJnt(q, q_dot, jntarraynull, base_vel, Twist::Zero(), wrenchnull, coriolis_torques, coriolis_base_wrench);
        
    }
    
       int TreeDynParam::JntToMass(const JntArray &q, FloatingJntSpaceInertiaMatrix& H)
    {
        //Check sizes when in debug mode
        if(q.rows()!=nj || H.rows()!=nj+6 || H.columns()!=nj+6 )
            return -1;
        
        return crba_floating_base_loop(undirected_tree,traversal,q,Ic,H);
        
    }
    
    TreeDynParam::~TreeDynParam()
    {
    }


}
}
