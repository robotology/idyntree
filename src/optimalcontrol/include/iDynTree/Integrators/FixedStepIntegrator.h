// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_FIXEDSTEPINTEGRATOR_H
#define IDYNTREE_OPTIMALCONTROL_FIXEDSTEPINTEGRATOR_H

#include <iDynTree/Integrator.h>
#include <iDynTree/VectorDynSize.h>

namespace iDynTree {
    namespace optimalcontrol {

        class DynamicalSystem;

        namespace integrators {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

            class FixedStepIntegrator : public Integrator
            {
            protected:

                virtual bool oneStepIntegration(double t0, double dT, const VectorDynSize& x0, VectorDynSize& x) = 0;

            public:
                FixedStepIntegrator();

                FixedStepIntegrator(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem);

                virtual ~FixedStepIntegrator();

                virtual bool integrate(double initialTime, double finalTime) override;

            };
        }
    }
}

#endif // IDYNTREE_OPTIMALCONTROL_FIXEDSTEPINTEGRATOR_H
