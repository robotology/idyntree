// Copyright  (C)  2009  Dominick Vanthienen <dominick dot vanthienen at mech dot kuleuven dot be>

/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <kdl_codyco/treedynparam.hpp>
//#include "frames_io.hpp"
//#include <iostream>

#include <kdl_codyco/regressor_utils.hpp>
#include <kdl_codyco/crba_loops.hpp>

#include <Eigen/Dense>


#ifndef NDEBUG
#include <iostream>
#endif

namespace KDL {
namespace CoDyCo {

    TreeDynParam::TreeDynParam(const Tree& _tree, Vector _grav, const TreeSerialization & _serialization):
        UndirectedTreeSolver(_tree,_serialization),
        nj(undirected_tree.getNrOfDOFs()),
        ns(undirected_tree.getNrOfLinks()),
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
