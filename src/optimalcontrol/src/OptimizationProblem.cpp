/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include "iDynTree/OptimizationProblem.h"
#include "iDynTree/Core/Utils.h"

namespace iDynTree {

    namespace optimization {

        OptimizationProblem::OptimizationProblem()
        {
        }

        OptimizationProblem::~OptimizationProblem()
        {
        }

        bool OptimizationProblem::prepare()
        {
            reportError("OptimizationProblem", "prepare", "Method not implemented.");
            return false;
        }

        void OptimizationProblem::reset()
        {

        }

        bool OptimizationProblem::getConstraintsBounds(VectorDynSize &constraintsLowerBounds, VectorDynSize &constraintsUpperBounds)
        {
            reportError("OptimizationProblem", "getConstraintsBounds", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::getVariablesUpperBound(VectorDynSize &variablesUpperBounds)
        {
            reportError("OptimizationProblem", "getVariablesUpperBounds", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::getVariablesLowerBound(VectorDynSize &variablesLowerBound)
        {
            reportError("OptimizationProblem", "getVariablesLowerBound", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::getConstraintsJacobianInfo(std::vector<size_t> &nonZeroElementRows, std::vector<size_t> &nonZeroElementColumns)
        {
            reportError("OptimizationProblem", "getConstraintsJacobianInfo", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::getHessianInfo(std::vector<size_t> &nonZeroElementRows, std::vector<size_t> &nonZeroElementColumns)
        {
            reportError("OptimizationProblem", "getHessianInfo", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::setVariables(const VectorDynSize &variables)
        {
            reportError("OptimizationProblem", "setVariables", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::evaluateCostFunction(double &costValue)
        {
            reportError("OptimizationProblem", "evaluateCostFunction", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::evaluateCostGradient(VectorDynSize &gradient)
        {
            reportError("OptimizationProblem", "evaluateCostGradient", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::evaluateCostHessian(MatrixDynSize &hessian)
        {
            reportError("OptimizationProblem", "evaluateCostHessian", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::evaluateConstraints(VectorDynSize &constraints)
        {
            reportError("OptimizationProblem", "evaluateConstraints", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::evaluateConstraintsJacobian(MatrixDynSize &jacobian)
        {
            reportError("OptimizationProblem", "evaluateConstraintsJacobian", "Method not implemented.");
            return false;
        }

        bool OptimizationProblem::evaluateConstraintsHessian(const VectorDynSize &constraintsMultipliers, MatrixDynSize &hessian)
        {
            reportError("OptimizationProblem", "evaluateConstraintsHessian", "Method not implemented.");
            return false;
        }

    }
}
