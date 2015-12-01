/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/JointState.h>

#include <iDynTree/Model/Model.h>

#include <iDynTree/Core/SpatialMotionVector.h>

namespace iDynTree
{

JointDoubleArray::JointDoubleArray(unsigned int nrOfDOFs): VectorDynSize(nrOfDOFs)
{

}

JointDoubleArray::JointDoubleArray(const iDynTree::Model& model): VectorDynSize(model.getNrOfDOFs())
{

}

void JointDoubleArray::resize(const iDynTree::Model& model)
{
    resize(model.getNrOfDOFs());
}

void JointDoubleArray::resize(unsigned int nrOfDOFs)
{
    VectorDynSize::resize(nrOfDOFs);
    this->zero();
}


JointDoubleArray::~JointDoubleArray()
{

}

DOFSpatialForceArray::DOFSpatialForceArray(unsigned int nrOfDOFs)
{
    resize(nrOfDOFs);
}

DOFSpatialForceArray::DOFSpatialForceArray(const iDynTree::Model& model)
{
    resize(model.getNrOfDOFs());
}

void DOFSpatialForceArray::resize(const iDynTree::Model& model)
{
    resize(model.getNrOfDOFs());
}

void DOFSpatialForceArray::resize(const unsigned int nrOfDOFs)
{
    this->m_dofSpatialForce.resize(nrOfDOFs,iDynTree::SpatialForceVector::Zero());
}


SpatialForceVector& DOFSpatialForceArray::operator()(const size_t dof)
{
    return this->m_dofSpatialForce[dof];
}

const SpatialForceVector& DOFSpatialForceArray::operator()(const size_t dof) const
{
    return this->m_dofSpatialForce[dof];
}

DOFSpatialForceArray::~DOFSpatialForceArray()
{

}


DOFSpatialMotionArray::DOFSpatialMotionArray(unsigned int nrOfDOFs)
{
    resize(nrOfDOFs);
}

DOFSpatialMotionArray::DOFSpatialMotionArray(const iDynTree::Model& model)
{
    resize(model.getNrOfDOFs());
}

void DOFSpatialMotionArray::resize(const iDynTree::Model& model)
{
    resize(model.getNrOfDOFs());
}

void DOFSpatialMotionArray::resize(const unsigned int nrOfDOFs)
{
    this->m_dofSpatialMotion.resize(nrOfDOFs,iDynTree::SpatialMotionVector::Zero());
}


SpatialMotionVector& DOFSpatialMotionArray::operator()(const size_t dof)
{
    return this->m_dofSpatialMotion[dof];
}

const SpatialMotionVector& DOFSpatialMotionArray::operator()(const size_t dof) const
{
    return this->m_dofSpatialMotion[dof];
}

DOFSpatialMotionArray::~DOFSpatialMotionArray()
{

}

}
