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

#include <kdl_codyco/treejnttocomjacsolver.hpp>
//#include "frames_io.hpp"
//#include <iostream>

#include <Eigen/LU>

namespace KDL {
namespace CoDyCo {

    TreeJntToCOMJacSolver::TreeJntToCOMJacSolver(const Tree& _tree, const TreeSerialization & _serialization):
        UndirectedTreeSolver(_tree,_serialization),
        Ic(undirected_tree.getNrOfLinks()),
        h_jac(undirected_tree.getNrOfDOFs()+6)
    {
    }

    int TreeJntToCOMJacSolver::JntToCOMJac(const JntArray& q_in, Jacobian& jac)
    {
        if( q_in.rows() != undirected_tree.getNrOfDOFs() ||
            jac.columns() != (undirected_tree.getNrOfDOFs() + 6) ) return -1;

        //Get centroidal momentum jacobian and com spatial inertia
        if( crba_momentum_jacobian_loop(undirected_tree,traversal,q_in,Ic,h_jac,I_com) != 0 ) return -2;

        //Divide the upper part (linear) for the mass, while multiply the lower part (angular) for the inverse of the inertia
        double m = I_com.getMass();
        RotationalInertia com_inertia = I_com.getRotationalInertia();
        jac.data.block(0,0,3,jac.data.cols()) = (1/m)*h_jac.data.block(0,0,3,h_jac.data.cols());
        jac.data.block(3,0,3,jac.data.cols()) = Eigen::Map<Eigen::Matrix3d>(com_inertia.data).inverse()*h_jac.data.block(3,0,3,h_jac.data.cols());

        return 0;
    }

    int TreeJntToCOMJacSolver::JntToMomentumJac(const JntArray& q_in, MomentumJacobian& jac)
    {
        if( q_in.rows() != undirected_tree.getNrOfDOFs() ||
            jac.columns() != (undirected_tree.getNrOfDOFs() + 6) ) return -1;

        //Get centroidal momentum jacobian and com spatial inertia
        if( crba_momentum_jacobian_loop(undirected_tree,traversal,q_in,Ic,jac,I_com) != 0 ) return -2;

        return 0;
    }


    TreeJntToCOMJacSolver::~TreeJntToCOMJacSolver()
    {
    }


}
}
