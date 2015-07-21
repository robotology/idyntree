/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_KDL_CONVERSIONS_HPP
#define IDYNTREE_KDL_CONVERSIONS_HPP

namespace KDL
{
    class Vector;
    class Rotation;
    class Frame;
    class Twist;
    class Wrench;
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
    class VectorDynSize;
    class RotationalInertiaRaw;
    class SpatialInertia;

    /**
     * iDynTree --> KDL conversions
     */
    KDL::Vector   ToKDL(const iDynTree::Position  & idyntree_position);
    KDL::Rotation ToKDL(const iDynTree::Rotation  & idyntree_rotation);
    KDL::Frame    ToKDL(const iDynTree::Transform & idyntree_transform);
    KDL::Twist    ToKDL(const iDynTree::Twist     & idyntree_twist);
    KDL::Wrench   ToKDL(const iDynTree::Wrench    & idyntree_wrench);
    bool          ToKDL(const iDynTree::VectorDynSize & idyntree_jntarray,
                        KDL::JntArray             & kdl_jntarray);

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

}



#endif
