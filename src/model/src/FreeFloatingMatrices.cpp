/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#include <iDynTree/Model/FreeFloatingMatrices.h>
#include <iDynTree/Model/Model.h>

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
