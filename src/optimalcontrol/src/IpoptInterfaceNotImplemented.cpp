// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/Optimizers/IpoptInterface.h>
#include <iDynTree/Utils.h>

using namespace iDynTree::optimization;
using namespace iDynTree;

IpoptInterface::IpoptInterface()
{

}

IpoptInterface::~IpoptInterface()
{

}

bool IpoptInterface::isAvailable() const
{
    return false;
}

bool IpoptInterface::setProblem(std::shared_ptr<OptimizationProblem> problem)
{
    reportError("IpoptInterface", "setProblem", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return false;
}

bool IpoptInterface::solve()
{
    reportError("IpoptInterface", "solve", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return false;
}

bool IpoptInterface::getPrimalVariables(VectorDynSize &primalVariables)
{
    reportError("IpoptInterface", "getPrimalVariables", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return false;
}

bool IpoptInterface::getDualVariables(VectorDynSize &constraintsMultipliers, VectorDynSize &lowerBoundsMultipliers, VectorDynSize &upperBoundsMultipliers)
{
    reportError("IpoptInterface", "getDualVariables", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return false;
}

bool IpoptInterface::getOptimalCost(double &optimalCost)
{
    reportError("IpoptInterface", "getOptimalCost", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return false;
}

bool IpoptInterface::getOptimalConstraintsValues(VectorDynSize &constraintsValues)
{
    reportError("IpoptInterface", "getOptimalConstraintsValues", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return false;
}

double IpoptInterface::minusInfinity()
{
    reportError("IpoptInterface", "minusInfinity", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return Optimizer::minusInfinity();
}

double IpoptInterface::plusInfinity()
{
    reportError("IpoptInterface", "plusInfinity", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return Optimizer::plusInfinity();
}

void IpoptInterface::useApproximatedHessians(bool useApproximatedHessian)
{
    reportError("IpoptInterface", "useApproximatedHessians", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
}

bool IpoptInterface::setIpoptOption(const std::string &tag, const std::string &value)
{
    reportError("IpoptInterface", "setIpoptOption", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return false;
}

bool IpoptInterface::setIpoptOption(const std::string &tag, double value)
{
    reportError("IpoptInterface", "setIpoptOption", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return false;
}

bool IpoptInterface::setIpoptOption(const std::string &tag, int value)
{
    reportError("IpoptInterface", "setIpoptOption", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return false;
}

bool IpoptInterface::getIpoptOption(const std::string &tag, std::string &value)
{
    reportError("IpoptInterface", "getIpoptOption", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return false;
}

bool IpoptInterface::getIpoptOption(const std::string &tag, double &value)
{
    reportError("IpoptInterface", "getIpoptOption", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return false;
}

bool IpoptInterface::getIpoptOption(const std::string &tag, int &value)
{
    reportError("IpoptInterface", "getIpoptOption", "IpoptInterface not implemented. Have you compiled iDynTree with the IDYNTREE_USES_IPOPT set to ON?");
    return false;
}


