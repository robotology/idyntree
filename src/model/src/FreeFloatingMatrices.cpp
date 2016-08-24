/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Model/FreeFloatingMatrices.h>
#include <iDynTree/Model/Model.h>

#include <cassert>

namespace iDynTree
{

FrameJacobian::FrameJacobian(size_t nrOfDofs): MatrixDynSize(6,6+nrOfDofs)
{
    MatrixDynSize::zero();
}


FrameJacobian::FrameJacobian(const Model& model)
{
    MatrixDynSize::resize(6,6+model.getNrOfDOFs());
    MatrixDynSize::zero();
}

bool FrameJacobian::isConsistent(const Model& model) const
{
    return (MatrixDynSize::rows() == 6 &&
            MatrixDynSize::cols() == 6+model.getNrOfDOFs());
}

void FrameJacobian::resize(const Model& model)
{
    MatrixDynSize::resize(6,6+model.getNrOfDOFs());
}

FrameJacobian::~FrameJacobian()
{

}

MomentumJacobian::MomentumJacobian(size_t nrOfDofs): MatrixDynSize(6,6+nrOfDofs)
{
    MatrixDynSize::zero();
}


MomentumJacobian::MomentumJacobian(const Model& model)
{
    MatrixDynSize::resize(6,6+model.getNrOfDOFs());
    MatrixDynSize::zero();
}

bool MomentumJacobian::isConsistent(const Model& model) const
{
    return (MatrixDynSize::rows() == 6 &&
            MatrixDynSize::cols() == 6+model.getNrOfDOFs());
}

void MomentumJacobian::resize(const Model& model)
{
    MatrixDynSize::resize(6,6+model.getNrOfDOFs());
}

MomentumJacobian::~MomentumJacobian()
{

}

FreeFloatingMassMatrix::FreeFloatingMassMatrix(size_t nrOfDofs): MatrixDynSize(6+nrOfDofs,6+nrOfDofs)
{
    MatrixDynSize::zero();
}


FreeFloatingMassMatrix::FreeFloatingMassMatrix(const Model& model)
{
    MatrixDynSize::resize(6+model.getNrOfDOFs(),6+model.getNrOfDOFs());
    MatrixDynSize::zero();
}

void FreeFloatingMassMatrix::resize(const Model& model)
{
    MatrixDynSize::resize(6+model.getNrOfDOFs(),6+model.getNrOfDOFs());
}

FreeFloatingMassMatrix::~FreeFloatingMassMatrix()
{

}

}
