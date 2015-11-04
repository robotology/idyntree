/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <kdl_codyco/KDLConversions.h>

#include <iDynTree/Core/ClassicalAcc.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/FreeFloatingState.h>


#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <kdl/rigidbodyinertia.hpp>

#include <Eigen/Dense>

#include <iostream>

#include <cstring>

namespace iDynTree
{

/**
 * Check that a given iDynTree::Model has a joint serialization that is compatible
 * with a KDL. This means that all joints have 1 DOFs or 0 DOFs, and in the serialization
 * the 1 DOFs joints came before the 0 DOFs one.
 */
bool checkModelJointSerializationIsCompatibleWithKDL(const Model & model)
{
    bool fixedJointsSerializationStarted = false;

    for(unsigned int jnt=0; jnt < model.getNrOfJoints(); jnt++)
    {
        IJointConstPtr jntPtr = model.getJoint(jnt);

        // Only simple 1 dofs joints (revolute, prismatic) are supported by KDL
        if( jntPtr->getNrOfDOFs() != jntPtr->getNrOfPosCoords() )
        {
            return false;
        }

        if( jntPtr->getNrOfDOFs() >= 2 )
        {
            return false;
        }

        // Return false if a 1-DOF joint is found after a fixed joint
        if( jntPtr->getNrOfDOFs() == 1 &&
            fixedJointsSerializationStarted )
        {
            return false;
        }

        if( jntPtr->getNrOfDOFs() == 0 )
        {
            fixedJointsSerializationStarted = true;
        }
    }

    return true;
}

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

    memcpy(kdl_twist.vel.data,idyntree_twist.getLinearVec3().data(),3*sizeof(double));
    memcpy(kdl_twist.rot.data,idyntree_twist.getAngularVec3().data(),3*sizeof(double));

    return kdl_twist;
}

KDL::Twist ToKDL(const SpatialAcc& idyntree_spatial_acc)
{
    KDL::Twist kdl_twist;

    memcpy(kdl_twist.vel.data,idyntree_spatial_acc.getLinearVec3().data(),3*sizeof(double));
    memcpy(kdl_twist.rot.data,idyntree_spatial_acc.getAngularVec3().data(),3*sizeof(double));

    return kdl_twist;
}


KDL::Twist ToKDL(const ClassicalAcc& idyntree_classical_acc)
{
    KDL::Twist kdl_twist;

    memcpy(kdl_twist.vel.data,idyntree_classical_acc.data(),3*sizeof(double));
    memcpy(kdl_twist.rot.data,idyntree_classical_acc.data()+3,3*sizeof(double));

    return kdl_twist;
}


KDL::Wrench ToKDL(const Wrench& idyntree_wrench)
{
    KDL::Wrench kdl_wrench;

    memcpy(kdl_wrench.force.data,idyntree_wrench.getLinearVec3().data(),3*sizeof(double));
    memcpy(kdl_wrench.torque.data,idyntree_wrench.getAngularVec3().data(),3*sizeof(double));

    return kdl_wrench;
}

KDL::Wrench ToKDL(const SpatialMomentum& idyntree_spatial_momentum)
{
    KDL::Wrench kdl_wrench;

    memcpy(kdl_wrench.force.data,idyntree_spatial_momentum.getLinearVec3().data(),3*sizeof(double));
    memcpy(kdl_wrench.torque.data,idyntree_spatial_momentum.getAngularVec3().data(),3*sizeof(double));

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

bool ToKDL(const iDynTree::FreeFloatingPos & idyntree_freeFloatingPos,
           KDL::Frame & kdl_world_H_base, KDL::JntArray& kdl_jntarray, std::vector<int> kdlDof2idyntree)
{
    kdl_world_H_base = ToKDL(idyntree_freeFloatingPos.worldBasePos());

    if( kdl_jntarray.rows() != idyntree_freeFloatingPos.getNrOfPosCoords() )
    {
         std::cerr << "[ERROR] ToKDL failed" << std::endl;
         return false;
    }

    // We do here the **strong** assumption that the dof index
    // of KDL and the one of iDynTree coincide
    for(unsigned int kdlDof=0; kdlDof < kdl_jntarray.rows(); kdlDof++ )
    {
        kdl_jntarray(kdlDof) = idyntree_freeFloatingPos.jointPos()((kdlDof2idyntree[kdlDof]));
    }
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

    memcpy(idyntree_twist.getLinearVec3().data(),kdl_twist.vel.data,3*sizeof(double));
    memcpy(idyntree_twist.getAngularVec3().data(),kdl_twist.rot.data,3*sizeof(double));

    return idyntree_twist;
}


Wrench ToiDynTree(const KDL::Wrench& kdl_wrench)
{
    Wrench idyntree_wrench;

    memcpy(idyntree_wrench.getLinearVec3().data(),kdl_wrench.force.data,3*sizeof(double));
    memcpy(idyntree_wrench.getAngularVec3().data(),kdl_wrench.torque.data,3*sizeof(double));

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

bool ToiDynTree(const KDL::Jacobian & kdl_jacobian, MatrixDynSize& idyntree_jacobian)
{
    idyntree_jacobian.resize(kdl_jacobian.rows(),kdl_jacobian.columns());

    Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(idyntree_jacobian.data(),idyntree_jacobian.rows(),idyntree_jacobian.cols())
        =  kdl_jacobian.data;

    return true;
}



}
