/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include "iDynTree/Optimizer.h"
#include "iDynTree/Core/Utils.h"

namespace iDynTree {
    namespace optimization {

        Optimizer::~Optimizer()
        {

        }

        bool Optimizer::setProblem(std::shared_ptr<OptimizationProblem> problem)
        {
            m_problem = problem;
            return true;
        }

        const std::weak_ptr<OptimizationProblem> Optimizer::problem() const
        {
            return m_problem;
        }

        bool Optimizer::getPrimalVariables(VectorDynSize &primalVariables)
        {
            reportError("Optimizer", "getPrimalVariables", "Method not implemented.");
            return false;
        }

        bool Optimizer::getDualVariables(VectorDynSize &dualVariables)
        {
            reportError("Optimizer", "getDualVariables", "Method not implemented.");
            return false;
        }

        double Optimizer::minusInfinity()
        {
            return -1e19;
        }

        double Optimizer::plusInfinity()
        {
            return 1e19;
        }

        bool Optimizer::getOptimalCost(double &optimalCost)
        {
            reportError("Optimizer", "getDualVariables", "Method not implemented.");
            return false;
        }

    }

}
