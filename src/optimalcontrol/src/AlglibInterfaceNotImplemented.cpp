/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/Optimizers/AlglibInterface.h>
#include <iDynTree/Core/Utils.h>

using namespace iDynTree::optimization;
using namespace iDynTree;

AlglibInterface::AlglibInterface()
{

}

AlglibInterface::~AlglibInterface()
{

}

bool AlglibInterface::isAvailable() const
{
    return false;
}

bool AlglibInterface::setProblem(std::shared_ptr<OptimizationProblem> problem)
{
    reportError("AlglibInterface", "setProblem", "AlglibInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_ALGLIB set to ON?");
    return false;
}

bool AlglibInterface::solve()
{
    reportError("AlglibInterface", "solve", "AlglibInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_ALGLIB set to ON?");
    return false;
}

bool AlglibInterface::getPrimalVariables(VectorDynSize &primalVariables)
{
    reportError("AlglibInterface", "getPrimalVariables", "AlglibInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_ALGLIB set to ON?");
    return false;
}

bool AlglibInterface::getDualVariables(VectorDynSize &constraintsMultipliers, VectorDynSize &lowerBoundsMultipliers, VectorDynSize &upperBoundsMultipliers)
{
    reportError("AlglibInterface", "getDualVariables", "AlglibInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_ALGLIB set to ON?");
    return false;
}

bool AlglibInterface::getOptimalCost(double &optimalCost)
{
    reportError("AlglibInterface", "getOptimalCost", "AlglibInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_ALGLIB set to ON?");
    return false;
}

bool AlglibInterface::getOptimalConstraintsValues(VectorDynSize &constraintsValues)
{
    reportError("AlglibInterface", "getOptimalConstraintsValues", "AlglibInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_ALGLIB set to ON?");
    return false;
}

double AlglibInterface::minusInfinity()
{
    reportError("AlglibInterface", "minusInfinity", "AlglibInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_ALGLIB set to ON?");
    return Optimizer::minusInfinity();
}

double AlglibInterface::plusInfinity()
{
    reportError("AlglibInterface", "plusInfinity", "AlglibInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_ALGLIB set to ON?");
    return Optimizer::plusInfinity();
}

bool AlglibInterface::setRHO(double rho)
{
    reportError("AlglibInterface", "setRHO", "AlglibInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_ALGLIB set to ON?");
    return false;
}

bool AlglibInterface::setOuterIterations(unsigned int outerIterations)
{
    reportError("AlglibInterface", "setOuterIterations", "AlglibInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_ALGLIB set to ON?");
    return false;
}

bool AlglibInterface::setStoppingCondition(double epsX)
{
    reportError("AlglibInterface", "setStoppingCondition", "AlglibInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_ALGLIB set to ON?");
    return false;
}

bool AlglibInterface::setMaximumIterations(unsigned int maxIter)
{
    reportError("AlglibInterface", "setMaximumIterations", "AlglibInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_ALGLIB set to ON?");
    return false;
}


