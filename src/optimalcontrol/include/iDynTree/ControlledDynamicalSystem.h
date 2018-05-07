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

#ifndef IDYNTREE_OPTIMALCONTROL_CONTROLLEDDYNAMICALSYSTEM_H
#define IDYNTREE_OPTIMALCONTROL_CONTROLLEDDYNAMICALSYSTEM_H

#include "iDynTree/DynamicalSystem.h"
#include "iDynTree/Controller.h"

#include <memory>

namespace iDynTree {
    class VectorDynSize;
    namespace optimalcontrol {

        class ControlledDynamicalSystem {
        public:
            ControlledDynamicalSystem();

            ControlledDynamicalSystem(const ControlledDynamicalSystem &other) = delete;

            ~ControlledDynamicalSystem();

            bool setDynamicalSystem(std::shared_ptr<DynamicalSystem> autonomousSystem, bool usePreviousController = false);

            bool setController(std::shared_ptr<Controller> controller);

            std::shared_ptr<DynamicalSystem> asDynamicalSystem();

        private:
            class ControlledDynamicalSystemImplementation;
            ControlledDynamicalSystemImplementation* m_pimpl;
        };
    }
}

#endif // IDYNTREE_OPTIMALCONTROL_CONTROLLEDDYNAMICALSYSTEM_H
