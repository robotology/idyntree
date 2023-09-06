// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/Optimizers/OsqpInterface.h>
#include <iDynTree/Utils.h>


using namespace iDynTree::optimization;
using namespace iDynTree;

OsqpInterface::OsqpInterface()
{ }

OsqpInterface::~OsqpInterface()
{ }

bool OsqpInterface::isAvailable() const
{
    return false;
}

bool OsqpInterface::setProblem(std::shared_ptr<OptimizationProblem> problem)
{
    reportError("OsqpInterface", "setProblem", "OsqpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_OSQPEIGEN set to ON?");
    return false;
}

bool OsqpInterface::solve()
{
    reportError("OsqpInterface", "solve", "OsqpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_OSQPEIGEN set to ON?");
    return false;
}

bool OsqpInterface::getPrimalVariables(VectorDynSize &primalVariables)
{
    reportError("OsqpInterface", "getPrimalVariables", "OsqpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_OSQPEIGEN set to ON?");
    return false;
}

bool OsqpInterface::getDualVariables(VectorDynSize &constraintsMultipliers, VectorDynSize &lowerBoundsMultipliers, VectorDynSize &upperBoundsMultipliers)
{
    reportError("OsqpInterface", "getDualVariables", "OsqpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_OSQPEIGEN set to ON?");
    return false;
}

bool OsqpInterface::getOptimalCost(double &optimalCost)
{
    reportError("OsqpInterface", "getOptimalCost", "OsqpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_OSQPEIGEN set to ON?");
    return false;
}

bool OsqpInterface::getOptimalConstraintsValues(VectorDynSize &constraintsValues)
{
    reportError("OsqpInterface", "getOptimalConstraintsValues", "OsqpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_OSQPEIGEN set to ON?");
    return false;
}

double OsqpInterface::minusInfinity()
{
    reportError("OsqpInterface", "setProblem", "OsqpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_OSQPEIGEN set to ON?");
    return Optimizer::minusInfinity();
}

double OsqpInterface::plusInfinity()
{
    reportError("OsqpInterface", "setProblem", "OsqpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_OSQPEIGEN set to ON?");
    return Optimizer::plusInfinity();
}

OsqpSettings &OsqpInterface::settings()
{
    reportError("OsqpInterface", "settings", "OsqpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_OSQPEIGEN set to ON?");
    OsqpSettings settings;
    return settings;
}


