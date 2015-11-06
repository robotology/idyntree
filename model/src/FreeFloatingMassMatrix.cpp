/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Model/FreeFloatingMassMatrix.h>
#include <iDynTree/Model/Model.h>

#include <cassert>

namespace iDynTree
{

FreeFloatingMassMatrix::FreeFloatingMassMatrix(size_t nrOfDofs): MatrixDynSize(6+nrOfDofs,6+nrOfDofs)
{
}


FreeFloatingMassMatrix::FreeFloatingMassMatrix(const Model& model)
{
    MatrixDynSize::resize(6+model.getNrOfDOFs(),6+model.getNrOfDOFs());
}

void FreeFloatingMassMatrix::resize(const Model& model)
{
    MatrixDynSize::resize(6+model.getNrOfDOFs(),6+model.getNrOfDOFs());
}

FreeFloatingMassMatrix::~FreeFloatingMassMatrix()
{

}

}
