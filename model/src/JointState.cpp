/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/Model.h>

namespace iDynTree
{

DOFDoubleArray::DOFDoubleArray(unsigned int nrOfDOFs): VectorDynSize(nrOfDOFs)
{

}

DOFDoubleArray::DOFDoubleArray(const iDynTree::Model& model): VectorDynSize(model.getNrOfDOFs())
{

}

void DOFDoubleArray::resize(const iDynTree::Model& model)
{
    resize(model.getNrOfDOFs());
}

void DOFDoubleArray::resize(unsigned int nrOfDOFs)
{
    VectorDynSize::resize(nrOfDOFs);
}


DOFDoubleArray::~DOFDoubleArray()
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
    this->m_dofSpatialForce.resize(nrOfDOFs);
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


}
