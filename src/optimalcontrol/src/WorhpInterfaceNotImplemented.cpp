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

#include <iDynTree/Optimizers/WorhpInterface.h>
#include <iDynTree/Core/Utils.h>

using namespace iDynTree::optimization;
using namespace iDynTree;


WorhpInterface::WorhpInterface()
{

}

WorhpInterface::~WorhpInterface()
{

}

bool WorhpInterface::isAvailable() const
{
    return false;
}

bool WorhpInterface::setProblem(std::shared_ptr<OptimizationProblem> problem)
{
    reportError("WorhpInterface", "setProblem", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return false;
}

bool WorhpInterface::solve()
{
    reportError("WorhpInterface", "solve", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return false;
}

bool WorhpInterface::getPrimalVariables(VectorDynSize &primalVariables)
{
    reportError("WorhpInterface", "getPrimalVariables", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return false;
}

bool WorhpInterface::getDualVariables(VectorDynSize &constraintsMultipliers, VectorDynSize &lowerBoundsMultipliers, VectorDynSize &upperBoundsMultipliers)
{
    reportError("WorhpInterface", "getDualVariables", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return false;
}

bool WorhpInterface::getOptimalCost(double &optimalCost)
{
    reportError("WorhpInterface", "getOptimalCost", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return false;
}

bool WorhpInterface::getOptimalConstraintsValues(VectorDynSize &constraintsValues)
{
    reportError("WorhpInterface", "getOptimalConstraintsValues", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return false;
}

double WorhpInterface::minusInfinity()
{
    reportError("WorhpInterface", "minusInfinity", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return Optimizer::minusInfinity();
}

double WorhpInterface::plusInfinity()
{
    reportError("WorhpInterface", "plusInfinity", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return Optimizer::plusInfinity();
}

void WorhpInterface::useApproximatedHessians(bool useApproximatedHessian)
{
    reportError("WorhpInterface", "useApproximatedHessians", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
}

bool WorhpInterface::setWorhpParam(const std::string &paramName, bool value)
{
    reportError("WorhpInterface", "setWorhpParam", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return false;
}

bool WorhpInterface::setWorhpParam(const std::string &paramName, double value)
{
    reportError("WorhpInterface", "setWorhpParam", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return false;
}

bool WorhpInterface::setWorhpParam(const std::string &paramName, int value)
{
    reportError("WorhpInterface", "setWorhpParam", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return false;
}

bool WorhpInterface::getWorhpParam(const std::string &paramName, bool &value)
{
    reportError("WorhpInterface", "getWorhpParam", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return false;
}

bool WorhpInterface::getWorhpParam(const std::string &paramName, double &value)
{
    reportError("WorhpInterface", "getWorhpParam", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return false;
}

bool WorhpInterface::getWorhpParam(const std::string &paramName, int &value)
{
    reportError("WorhpInterface", "getWorhpParam", "WorhpInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_WORHP set to ON?");
    return false;
}

