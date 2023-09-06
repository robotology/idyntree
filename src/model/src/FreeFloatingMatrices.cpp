// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <iDynTree/FreeFloatingMatrices.h>
#include <iDynTree/Model.h>

#include <cassert>

namespace iDynTree
{

FrameFreeFloatingJacobian::FrameFreeFloatingJacobian(size_t nrOfDofs): MatrixDynSize(6,6+nrOfDofs)
{
    MatrixDynSize::zero();
}


FrameFreeFloatingJacobian::FrameFreeFloatingJacobian(const Model& model)
{
    MatrixDynSize::resize(6,6+model.getNrOfDOFs());
    MatrixDynSize::zero();
}

bool FrameFreeFloatingJacobian::isConsistent(const Model& model) const
{
    return (MatrixDynSize::rows() == 6 &&
            MatrixDynSize::cols() == 6+model.getNrOfDOFs());
}

void FrameFreeFloatingJacobian::resize(const Model& model)
{
    MatrixDynSize::resize(6,6+model.getNrOfDOFs());
}

FrameFreeFloatingJacobian::~FrameFreeFloatingJacobian()
{

}

MomentumFreeFloatingJacobian::MomentumFreeFloatingJacobian(size_t nrOfDofs): MatrixDynSize(6,6+nrOfDofs)
{
    MatrixDynSize::zero();
}


MomentumFreeFloatingJacobian::MomentumFreeFloatingJacobian(const Model& model)
{
    MatrixDynSize::resize(6,6+model.getNrOfDOFs());
    MatrixDynSize::zero();
}

bool MomentumFreeFloatingJacobian::isConsistent(const Model& model) const
{
    return (MatrixDynSize::rows() == 6 &&
            MatrixDynSize::cols() == 6+model.getNrOfDOFs());
}

void MomentumFreeFloatingJacobian::resize(const Model& model)
{
    MatrixDynSize::resize(6,6+model.getNrOfDOFs());
}

MomentumFreeFloatingJacobian::~MomentumFreeFloatingJacobian()
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
