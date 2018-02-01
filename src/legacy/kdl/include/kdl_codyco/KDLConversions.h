/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_KDL_CONVERSIONS_HPP
#define IDYNTREE_KDL_CONVERSIONS_HPP


#include <vector>

namespace KDL
{
    class Vector;
    class Rotation;
    class Frame;
    class Twist;
    class Wrench;
    class Jacobian;
    class JntArray;
    class RotationalInertia;
    class RigidBodyInertia;
}

namespace iDynTree
{
    class Position;
    class Rotation;
    class Transform;
    class Twist;
    class Wrench;
    class MatrixDynSize;
    class VectorDynSize;
    class RotationalInertiaRaw;
    class SpatialInertia;
    class SpatialAcc;
    class SpatialMomentum;
    class ClassicalAcc;
    class FreeFloatingPos;
    class FreeFloatingVel;
    class FreeFloatingAcc;

    /**
     * iDynTree --> KDL conversions
     */
    KDL::Vector   ToKDL(const iDynTree::Position  & idyntree_position);
    KDL::Rotation ToKDL(const iDynTree::Rotation  & idyntree_rotation);
    KDL::Frame    ToKDL(const iDynTree::Transform & idyntree_transform);
    KDL::Twist    ToKDL(const iDynTree::Twist     & idyntree_twist);
    KDL::Wrench   ToKDL(const iDynTree::Wrench    & idyntree_wrench);
    KDL::Twist    ToKDL(const iDynTree::SpatialAcc & idyntree_classical_acc);
    KDL::Twist    ToKDL(const iDynTree::ClassicalAcc & idyntree_classical_acc);
    KDL::Wrench   ToKDL(const iDynTree::SpatialMomentum    & idyntree_spatial_momentum);
    bool          ToKDL(const iDynTree::VectorDynSize & idyntree_jntarray,
                               KDL::JntArray             & kdl_jntarray);

    /**
     * Convert the configuration of a free floating robot from iDynTree.
     * Considering that the joint serialization between the two can be different,
     * a map for connecting the two is also passed.
     */
    bool ToKDL(const iDynTree::FreeFloatingPos & idyntree_freeFloatingPos,
               KDL::Frame & world_H_base, KDL::JntArray& kdl_jntarray, const std::vector<int> kdlDof2idyntree);

    /**
     * Convert the position, velocity and configuration of a free floating robot from iDynTree
     * to KDL.
     * Considering that the joint serialization between the two can be different,
     * a map for connecting the two is also passed.
     */
    bool ToKDL(const iDynTree::FreeFloatingVel & idyntree_freeFloatingVel,
               KDL::Twist & base_vel,     KDL::JntArray& kdl_jntVel,
               const std::vector<int> kdlDof2idyntree);

    bool ToKDL(const iDynTree::FreeFloatingAcc & idyntree_freeFloatingAcc,
               KDL::Twist & base_acc,     KDL::JntArray& kdl_jntacc,
               const std::vector<int> kdlDof2idyntree);

    /**
     * KDL --> iDynTree conversions
     */
    iDynTree::Position   ToiDynTree(const KDL::Vector & kdl_vector);
    iDynTree::Rotation   ToiDynTree(const KDL::Rotation & kdl_rotation);
    iDynTree::Transform  ToiDynTree(const KDL::Frame & kdl_transform);
    iDynTree::Twist      ToiDynTree(const KDL::Twist & kdl_twist);
    iDynTree::Wrench     ToiDynTree(const KDL::Wrench & kdl_wrench);
    iDynTree::RotationalInertiaRaw  ToiDynTree(const KDL::RotationalInertia & kdl_rotInertia);
    iDynTree::SpatialInertia ToiDynTree(const KDL::RigidBodyInertia & kdl_inertia);
    bool                 ToiDynTree(const KDL::JntArray  & kdl_jntarray,
                                    iDynTree::VectorDynSize & idyntree_jntarray);
    bool ToiDynTree(const KDL::Jacobian & kdl_jacobian, iDynTree::MatrixDynSize& idyntree_jacobian);


}



#endif
