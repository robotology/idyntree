/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Model/JointState.h>

#include <iDynTree/Model/Model.h>

#include <iDynTree/Core/SpatialMotionVector.h>

namespace iDynTree
{

JointPosDoubleArray::JointPosDoubleArray(std::size_t nrOfPosCoords): VectorDynSize(nrOfPosCoords)
{

}

JointPosDoubleArray::JointPosDoubleArray(const iDynTree::Model& model): VectorDynSize(model.getNrOfPosCoords())
{

}

void JointPosDoubleArray::resize(const iDynTree::Model& model)
{
    resize(model.getNrOfPosCoords());
}

void JointPosDoubleArray::resize(std::size_t nrOfPosCoords)
{
    VectorDynSize::resize(nrOfPosCoords);
    this->zero();
}

bool JointPosDoubleArray::isConsistent(const Model& model) const
{
    return (this->size() == model.getNrOfPosCoords());
}

JointPosDoubleArray::~JointPosDoubleArray()
{

}


JointDOFsDoubleArray::JointDOFsDoubleArray(std::size_t nrOfDOFs): VectorDynSize(nrOfDOFs)
{

}

JointDOFsDoubleArray::JointDOFsDoubleArray(const iDynTree::Model& model): VectorDynSize(model.getNrOfDOFs())
{

}

void JointDOFsDoubleArray::resize(const iDynTree::Model& model)
{
    resize(model.getNrOfDOFs());
}

void JointDOFsDoubleArray::resize(std::size_t nrOfDOFs)
{
    VectorDynSize::resize(nrOfDOFs);
    this->zero();
}

bool JointDOFsDoubleArray::isConsistent(const Model& model) const
{
    return (this->size() == model.getNrOfDOFs());
}

JointDOFsDoubleArray::~JointDOFsDoubleArray()
{

}

DOFSpatialForceArray::DOFSpatialForceArray(std::size_t nrOfDOFs)
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

void DOFSpatialForceArray::resize(const std::size_t nrOfDOFs)
{
    this->m_dofSpatialForce.resize(nrOfDOFs,iDynTree::SpatialForceVector::Zero());
}

bool DOFSpatialForceArray::isConsistent(const Model& model) const
{
    return (this->m_dofSpatialForce.size() == model.getNrOfDOFs());
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


DOFSpatialMotionArray::DOFSpatialMotionArray(std::size_t nrOfDOFs)
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

bool DOFSpatialMotionArray::isConsistent(const Model& model) const
{
    return (this->m_dofSpatialMotion.size() == model.getNrOfDOFs());
}

void DOFSpatialMotionArray::resize(const std::size_t nrOfDOFs)
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
