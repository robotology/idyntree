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

#ifndef IDYNTREE_OPTIMALCONTROL_FIXEDSTEPINTEGRATOR_H
#define IDYNTREE_OPTIMALCONTROL_FIXEDSTEPINTEGRATOR_H

#include <iDynTree/Integrator.h>
#include <iDynTree/Core/VectorDynSize.h>

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
