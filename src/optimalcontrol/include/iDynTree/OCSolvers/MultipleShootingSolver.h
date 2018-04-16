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

#ifndef IDYNTREE_OPTIMALCONTROL_MULTIPLESHOOTINGSOLVER_H
#define IDYNTREE_OPTIMALCONTROL_MULTIPLESHOOTINGSOLVER_H

#include "iDynTree/OptimalControlSolver.h"
#include "iDynTree/OptimizationProblem.h"
#include "iDynTree/Optimizer.h"

#include <vector>
#include <memory>

namespace iDynTree {

    class VectorDynSize;

    namespace optimalcontrol {

        class OptimalControlProblem;

        namespace integrators {
            class Integrator;
        }
        using namespace integrators;

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class MultipleShootingTranscription : public optimization::OptimizationProblem {

            friend class MultipleShootingSolver;

            MultipleShootingTranscription();

            MultipleShootingTranscription(const std::shared_ptr<OptimalControlProblem> problem, const std::shared_ptr<Integrator> integrationMethod);

            MultipleShootingTranscription(const MultipleShootingTranscription& other) = delete;

            size_t setControlMeshPoints();

            bool setMeshPoints();

            bool setOptimalControlProblem(const std::shared_ptr<OptimalControlProblem> problem);

            bool setIntegrator(const std::shared_ptr<Integrator> integrationMethod);

            bool setStepSizeBounds(const double minStepSize, const double maxStepsize);

            bool setControlPeriod(double period);

            bool setAdditionalStateMeshPoints(const std::vector<double>& stateMeshes);

            bool setAdditionalControlMeshPoints(const std::vector<double>& controlMeshes);

            void setPlusInfinity(double plusInfinity);

            void setMinusInfinity(double minusInfinity);

            class MultipleShootingTranscriptionPimpl;
            MultipleShootingTranscriptionPimpl *m_pimpl;

        public:

            virtual ~MultipleShootingTranscription() override;

            virtual bool prepare() override;

            virtual void reset() override;

            virtual unsigned int numberOfVariables() override;

            virtual bool getConstraintsInfo(unsigned int& numberOfConstraints, VectorDynSize& constraintsLowerBounds, VectorDynSize& constraintsUpperBounds) override;

//            virtual bool getVariablesUpperBound(VectorDynSize& variablesUpperBound);

//            virtual bool getVariablesLowerBound(VectorDynSize& variablesLowerBound);

//            virtual bool getConstraintsJacobianInfo(std::vector<size_t>& nonZeroElementRows, std::vector<size_t>& nonZeroElementColumns);

//            virtual bool getHessianInfo(std::vector<size_t>& nonZeroElementRows, std::vector<size_t>& nonZeroElementColumns);

//            virtual bool setVariables(const VectorDynSize& variables);

//            virtual bool evaluateCostFunction(double& costValue);

//            virtual bool evaluateCostGradient(VectorDynSize& gradient);

//            virtual bool evaluateCostHessian(MatrixDynSize& hessian); //using dense matrices, but the sparsity pattern is still obtained

//            virtual bool evaluateConstraints(VectorDynSize& constraints);

//            virtual bool evaluateConstraintsJacobian(MatrixDynSize& jacobian); //using dense matrices, but the sparsity pattern is still obtained

//            virtual bool evaluateConstraintsHessian(const VectorDynSize& constraintsMultipliers, MatrixDynSize& hessian); //using dense matrices, but the sparsity pattern is still obtained

        };


        class MultipleShootingSolver : public OptimalControlSolver {

        public:
            MultipleShootingSolver(const std::shared_ptr<OptimalControlProblem>& ocProblem);

            MultipleShootingSolver(const MultipleShootingSolver& other) = delete;

            bool setStepSizeBounds(double minStepSize, double maxStepsize);

            bool setIntegrator(const std::shared_ptr<Integrator> integrationMethod);

            bool setControlPeriod(double period);

            bool setAdditionalStateMeshPoints(const std::vector<double>& stateMeshes);

            bool setAdditionalControlMeshPoints(const std::vector<double>& controlMeshes);

            bool setOptimizer(std::shared_ptr<optimization::Optimizer> optimizer);



            // FIXME: These two cannot be used as VectorDynTree
            // as they are trajectories, not single vectors
            void setInitialGuess(const iDynTree::VectorDynSize& initialGuess);
            const iDynTree::VectorDynSize& lastSolution();


            virtual bool initialize() override;
            virtual bool solve() override;

        private:
            std::shared_ptr<MultipleShootingTranscription> m_transcription;
            std::shared_ptr<optimization::Optimizer> m_optimizer;
        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_MULTIPLESHOOTINGSOLVER_H */
