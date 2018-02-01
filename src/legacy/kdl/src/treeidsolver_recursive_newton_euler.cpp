// Copyright (C)   2013 Silvio Traversaro - CoDyCo project http://www.codyco.eu/

// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

///  This and the source file were modified from ChainIdSolver_recursive_newton_euler.h/cpp
///  Thus, still probably covered by LGPL, which is why I build it as a library and then link to it
///     Darren Earl, HRL 6/2012

//versione extracted from r2_controllers

#include <kdl_codyco/treeidsolver_recursive_newton_euler.hpp>
#include <kdl_codyco/utils.hpp>
#include <kdl_codyco/rnea_loops.hpp>

#include <kdl/frames_io.hpp>
#include <deque>
using namespace std;

namespace KDL{
namespace CoDyCo {


    TreeIdSolver_RNE::TreeIdSolver_RNE(const Tree& tree, Vector grav, const TreeSerialization & serialization)
    : UndirectedTreeSolver(tree,serialization)
    {

        //Initializing gravitational acceleration (if any)
        ag=-Twist(grav,Vector::Zero());

        //allocate vectors
        v.resize(tree.getNrOfSegments(),Twist::Zero());
        a.resize(tree.getNrOfSegments(),Twist::Zero());
        f.resize(tree.getNrOfSegments(),Wrench::Zero());

    }

    int TreeIdSolver_RNE::CartToJnt(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques)
    {
        Wrench dummy;
        return CartToJnt(q,q_dot,q_dotdot,Twist::Zero(),ag,f_ext,torques,dummy);
    }


    int TreeIdSolver_RNE::CartToJnt(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const Twist& base_velocity, const Twist& base_acceleration, const Wrenches& f_ext,JntArray &torques, Wrench& base_force)
    {
        if( q.rows() != undirected_tree.getNrOfDOFs() ||
            q_dot.rows() != q.rows()                  ||
            q_dotdot.rows() != q.rows()               ||
            f_ext.size() != undirected_tree.getNrOfLinks() ||
            torques.rows() != q.rows() ) return -1;

        base_force = Wrench::Zero();

        rneaKinematicLoop(undirected_tree,q,q_dot,q_dotdot,traversal,base_velocity,base_acceleration,v,a);

        rneaDynamicLoop(undirected_tree,q,traversal,v,a,f_ext,f,torques,base_force);

        return 0;
    }


}
}//namespace
