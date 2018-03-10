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

#include "iDynTree/Integrators/ImplicitTrapezoidal.h"
#include "iDynTree/Core/Utils.h"

namespace iDynTree {
    namespace optimalcontrol {
        namespace integrators {

            bool ImplicitTrapezoidal::oneStepIntegration(double t0, double dT, const iDynTree::VectorDynSize &x0, iDynTree::VectorDynSize &x)
            {
                reportError(m_info.name().c_str(), "oneStepIntegration", "The ImplicitTrapezoidal method has not been implemented to integrate a dynamical system yet.");
            }

            ImplicitTrapezoidal::ImplicitTrapezoidal(const std::shared_ptr<DynamicalSystem> dynamicalSystem) : FixedStepIntegrator(dynamicalSystem)
            {
                m_infoData->name = "ImplicitTrapezoidal";
                m_infoData->isExplicit = false;
                m_infoData->numberOfStages = 0;
            }

            ImplicitTrapezoidal::~ImplicitTrapezoidal()
            {}


        }
    }
}
