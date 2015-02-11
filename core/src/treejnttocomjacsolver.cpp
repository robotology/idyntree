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

#include "treejnttocomjacsolver.hpp"
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
