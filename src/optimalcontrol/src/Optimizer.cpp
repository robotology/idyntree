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

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Optimizer.h>
#include <iDynTree/Core/Utils.h>

namespace iDynTree {
    namespace optimization {

        Optimizer::Optimizer()
        {

        }

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

        bool Optimizer::getDualVariables(VectorDynSize &constraintsMultipliers, VectorDynSize &lowerBoundsMultipliers, VectorDynSize &upperBoundsMultipliers)
        {
            reportError("Optimizer", "getDualVariables", "Method not implemented.");
            return false;
        }

        double Optimizer::minusInfinity()
        {
            return -1E19;
        }

        double Optimizer::plusInfinity()
        {
            return 1E19;
        }

        bool Optimizer::getOptimalCost(double &optimalCost)
        {
            reportError("Optimizer", "getDualVariables", "Method not implemented.");
            return false;
        }

        bool Optimizer::getOptimalConstraintsValues(VectorDynSize &constraintsValues)
        {
            reportError("Optimizer", "getOptimalConstraintsValues", "Method not implemented.");
            return false;
        }

    }

}
