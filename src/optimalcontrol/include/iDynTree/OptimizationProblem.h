/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_OPTIMIZATIONPROBLEM_H
#define IDYNTREE_OPTIMALCONTROL_OPTIMIZATIONPROBLEM_H

#include <vector>
#include <cstddef>

namespace iDynTree {

    class VectorDynSize;

    class MatrixDynSize;

    namespace optimization {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class OptimizationProblem {

        public:

            OptimizationProblem();

            virtual ~OptimizationProblem();

            virtual bool prepare();

            virtual void reset();

            virtual unsigned int numberOfVariables() = 0;

            virtual unsigned int numberOfConstraints() = 0;

            virtual bool getConstraintsBounds(VectorDynSize& constraintsLowerBounds, VectorDynSize& constraintsUpperBounds);

            virtual bool getVariablesUpperBound(VectorDynSize& variablesUpperBound); //return false if not upper bounded

            virtual bool getVariablesLowerBound(VectorDynSize& variablesLowerBound); //return false if not lower bounded

            virtual bool getConstraintsJacobianInfo(std::vector<size_t>& nonZeroElementRows, std::vector<size_t>& nonZeroElementColumns);

            virtual bool getHessianInfo(std::vector<size_t>& nonZeroElementRows, std::vector<size_t>& nonZeroElementColumns); //costs and constraints together

           /* virtual bool getStartingPoint(VectorDynSize& variablesGuess,
                                          VectorDynSize& lowerBoundsMultipliersGuess,
                                          VectorDynSize& upperBoundsMultipliersGuess,
                                          VectorDynSize& constraintsMultiplierGuess); */ //This method should be part of the solver interface

            virtual bool setVariables(const VectorDynSize& variables);

            virtual bool evaluateCostFunction(double& costValue);

            virtual bool evaluateCostGradient(VectorDynSize& gradient);

            virtual bool evaluateCostHessian(MatrixDynSize& hessian); //using dense matrices, but the sparsity pattern is still obtained

            virtual bool evaluateConstraints(VectorDynSize& constraints);

            virtual bool evaluateConstraintsJacobian(MatrixDynSize& jacobian); //using dense matrices, but the sparsity pattern is still obtained

            virtual bool evaluateConstraintsHessian(const VectorDynSize& constraintsMultipliers, MatrixDynSize& hessian); //using dense matrices, but the sparsity pattern is still obtained

        };
    }
}

#endif // IDYNTREE_OPTIMALCONTROL_OPTIMIZATIONPROBLEM_H
