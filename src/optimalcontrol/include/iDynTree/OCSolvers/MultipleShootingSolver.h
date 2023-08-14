// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_MULTIPLESHOOTINGSOLVER_H
#define IDYNTREE_OPTIMALCONTROL_MULTIPLESHOOTINGSOLVER_H

#include <iDynTree/OptimalControlSolver.h>
#include <iDynTree/TimeVaryingObject.h>
#include <iDynTree/Optimizer.h>

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

            bool setInitialState(const VectorDynSize &initialState);

            bool setGuesses(std::shared_ptr<optimalcontrol::TimeVaryingVector> stateGuesses,
                            std::shared_ptr<optimalcontrol::TimeVaryingVector> controlGuesses);

            bool getTimings(std::vector<double>& stateEvaluations, std::vector<double>& controlEvaluations);

            bool getPossibleTimings(std::vector<double>& stateEvaluations, std::vector<double>& controlEvaluations);

            virtual bool solve() override;

            bool getSolution(std::vector<VectorDynSize>& states, std::vector<VectorDynSize>& controls);

            void resetTranscription();

            void addConstraintsHessianRegularization(double regularization);

            void disableConstraintsHessianRegularization();

            void addCostsHessianRegularization(double regularization);

            void disableCostsHessianRegularization();

        private:

            class MultipleShootingTranscription;

            std::shared_ptr<MultipleShootingTranscription> m_transcription;
            std::shared_ptr<optimization::Optimizer> m_optimizer;
        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_MULTIPLESHOOTINGSOLVER_H */
