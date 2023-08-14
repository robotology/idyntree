// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_OPTIMIZATIONPROBLEM_H
#define IDYNTREE_OPTIMALCONTROL_OPTIMIZATIONPROBLEM_H

#include <vector>
#include <memory>
#include <cstddef>

namespace iDynTree {

    class VectorDynSize;

    class MatrixDynSize;

    namespace optimization {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class OptimizationProblemInfoData {
        protected:
            friend class OptimizationProblem;
            OptimizationProblemInfoData();
        public:
            bool hasLinearConstraints;

            bool hasNonLinearConstraints;

            bool costIsLinear;

            bool costIsQuadratic;

            bool costIsNonLinear;

            bool hasSparseConstraintJacobian;

            bool hasSparseHessian;

            bool hessianIsProvided;
        };

        class OptimizationProblemInfo {
        private:
            std::shared_ptr<OptimizationProblemInfoData> m_data;
        public:
            OptimizationProblemInfo(std::shared_ptr<OptimizationProblemInfoData> data);

            OptimizationProblemInfo() = delete;

            OptimizationProblemInfo(const OptimizationProblemInfo &other) = delete;

            bool hasLinearConstraints() const;

            bool hasNonLinearConstraints() const;

            bool costIsLinear() const;

            bool costIsQuadratic() const;

            bool costIsNonLinear() const;

            bool hasSparseConstraintJacobian() const;

            bool hasSparseHessian() const;

            bool hessianIsProvided() const;
        };

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

            virtual bool getGuess(VectorDynSize &guess);

            virtual bool setVariables(const VectorDynSize& variables);

            virtual bool evaluateCostFunction(double& costValue);

            virtual bool evaluateCostGradient(VectorDynSize& gradient); //for quadratic costs this corresponds to Hx + g !

            virtual bool evaluateCostHessian(MatrixDynSize& hessian); //using dense matrices, but the sparsity pattern is still obtained. Initialize hessian to zero in case of dense solvers

            virtual bool evaluateConstraints(VectorDynSize& constraints);

            virtual bool evaluateConstraintsJacobian(MatrixDynSize& jacobian); //using dense matrices, but the sparsity pattern is still obtained

            virtual bool evaluateConstraintsHessian(const VectorDynSize& constraintsMultipliers, MatrixDynSize& hessian); //using dense matrices, but the sparsity pattern is still obtained

            const OptimizationProblemInfo& info() const;

        protected:
            std::shared_ptr<OptimizationProblemInfoData> m_infoData;
            OptimizationProblemInfo m_info;
        };
    }
}

#endif // IDYNTREE_OPTIMALCONTROL_OPTIMIZATIONPROBLEM_H
