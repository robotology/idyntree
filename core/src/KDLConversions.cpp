/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "KDLConversions.h"

#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Core/VectorDynSize.h>


#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl/rigidbodyinertia.hpp>

#include <Eigen/Dense>

#include <iostream>

#include <cstring>

namespace iDynTree
{

KDL::Vector ToKDL(const Position& idyntree_position)
{
    KDL::Vector kdl_vector;

    memcpy(kdl_vector.data,idyntree_position.data(),3*sizeof(double));

    return kdl_vector;
}

KDL::Rotation ToKDL(const Rotation& idyntree_rotation)
{
    KDL::Rotation kdl_rotation;

    memcpy(kdl_rotation.data,idyntree_rotation.data(),9*sizeof(double));

    return kdl_rotation;
}


KDL::Frame ToKDL(const Transform& idyntree_transform)
{
    KDL::Frame kdl_frame;

    kdl_frame.M = ToKDL(idyntree_transform.getRotation());
    kdl_frame.p = ToKDL(idyntree_transform.getPosition());

    return kdl_frame;
}

KDL::Twist ToKDL(const Twist& idyntree_twist)
{
    KDL::Twist kdl_twist;

    memcpy(kdl_twist.vel.data,idyntree_twist.data(),3*sizeof(double));
    memcpy(kdl_twist.rot.data,idyntree_twist.data()+3,3*sizeof(double));

    return kdl_twist;
}

KDL::Wrench ToKDL(const Wrench& idyntree_wrench)
{
    KDL::Wrench kdl_wrench;

    memcpy(kdl_wrench.force.data,idyntree_wrench.data(),3*sizeof(double));
    memcpy(kdl_wrench.torque.data,idyntree_wrench.data()+3,3*sizeof(double));

    return kdl_wrench;
}

bool ToKDL(const VectorDynSize& idyntree_jntarray, KDL::JntArray& kdl_jntarray)
{
    if( kdl_jntarray.rows() != idyntree_jntarray.size() )
    {
        std::cerr << "[ERROR] ToKDL failed" << std::endl;
        return false;
    }

    kdl_jntarray.data =
        Eigen::Map<const Eigen::VectorXd>(idyntree_jntarray.data(), idyntree_jntarray.size());

    return true;
}


Position ToiDynTree(const KDL::Vector& kdl_vector)
{
    Position idyntree_position;

    memcpy(idyntree_position.data(),kdl_vector.data,3*sizeof(double));

    return idyntree_position;
}


Rotation ToiDynTree(const KDL::Rotation& kdl_rotation)
{
    Rotation idyntree_rotation;

    memcpy(idyntree_rotation.data(),kdl_rotation.data,9*sizeof(double));

    return idyntree_rotation;
}


Transform ToiDynTree(const KDL::Frame& kdl_frame)
{
    Transform idyntree_transform;

    idyntree_transform.setPosition(ToiDynTree(kdl_frame.p));
    idyntree_transform.setRotation(ToiDynTree(kdl_frame.M));

    return idyntree_transform;
}


Twist ToiDynTree(const KDL::Twist& kdl_twist)
{
    Twist idyntree_twist;

    memcpy(idyntree_twist.data(),kdl_twist.vel.data,3*sizeof(double));
    memcpy(idyntree_twist.data()+3,kdl_twist.rot.data,3*sizeof(double));

    return idyntree_twist;
}


Wrench ToiDynTree(const KDL::Wrench& kdl_wrench)
{
    Wrench idyntree_wrench;

    memcpy(idyntree_wrench.data(),kdl_wrench.force.data,3*sizeof(double));
    memcpy(idyntree_wrench.data()+3,kdl_wrench.torque.data,3*sizeof(double));

    return idyntree_wrench;
}

iDynTree::RotationalInertiaRaw  ToiDynTree(const KDL::RotationalInertia & kdl_rotInertia)
{
    iDynTree::RotationalInertiaRaw ret;
    // \todo TODO migrate this method to RotationalInertia class when available
    // the rotational inertia matrix is symmetric, so the column/row major ordering
    // does not matter (but in general in iDynTree we assume the RowMajor ordering
    Eigen::Map<Eigen::Matrix3d> idynTreeInertia(ret.data());
    Eigen::Map<const Eigen::Matrix3d> kdlInertia(kdl_rotInertia.data);

    idynTreeInertia = kdlInertia;

    return ret;
}


SpatialInertia ToiDynTree(const KDL::RigidBodyInertia& kdl_inertia)
{
    double mass_idyntree = kdl_inertia.getMass();
    iDynTree::Position com_idyntree = ToiDynTree(kdl_inertia.getCOG());
    // This works because both kdl_inertia.getRotationalInertia() and the iDynTree::SpatialInertia
    // constructor consider the RotationalInertia expressed wrt the origin of the link
    // differently from the KDL::RigidBodyInertia construtor that takes the inertia expressed
    // with respect to the center of mass
    iDynTree::RotationalInertiaRaw rotInertia_idyntree = ToiDynTree(kdl_inertia.getRotationalInertia());

    return iDynTree::SpatialInertia(mass_idyntree,com_idyntree,rotInertia_idyntree);
}

bool ToiDynTree(const KDL::JntArray& kdl_jntarray, VectorDynSize& idyntree_jntarray)
{
    if( kdl_jntarray.rows() != idyntree_jntarray.size() )
    {
        std::cerr << "[ERROR] ToiDynTree failed" << std::endl;
        return false;
    }

    Eigen::Map<Eigen::VectorXd>(idyntree_jntarray.data(),idyntree_jntarray.size())
        =  kdl_jntarray.data;

    return true;
}



}
