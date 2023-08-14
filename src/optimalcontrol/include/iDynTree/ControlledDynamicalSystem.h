// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_CONTROLLEDDYNAMICALSYSTEM_H
#define IDYNTREE_OPTIMALCONTROL_CONTROLLEDDYNAMICALSYSTEM_H

#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Controller.h>

#include <memory>

namespace iDynTree {
    class VectorDynSize;
    namespace optimalcontrol {

        /**
         * @brief The ControlledDynamicalSystem class allows to easily connect a DynamicalSystem with a Controller.
         * It defines a controlled dynamical system that can be integrated using an Integrator class.
         */


        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */
        class ControlledDynamicalSystem {
        public:
            ControlledDynamicalSystem();

            ControlledDynamicalSystem(const ControlledDynamicalSystem &other) = delete;

            ~ControlledDynamicalSystem();

            /**
             * @brief Set the DynamicalSystem pointer.
             * The usePreviousController flag allows to use again the controller stored in memory after a previous call to setController. This allow to call the method setController before setting the dynamical system.
             * @param[in] autonomousSystem The DynamicalSystem pointer.
             * @param usePreviousController A flag specifying whether the previously specified controller should be used. Default is false.
             * @return True if successfull. Possible sources of failures: empty pointer, usePreviouController is true and the dynamicalSystem and the previous controller are not compatible.
             */
            bool setDynamicalSystem(std::shared_ptr<DynamicalSystem> autonomousSystem, bool usePreviousController = false);

            /**
             * @brief Set the controller pointer.
             * @param[in] controller Controller pointer.
             * @return True if successfull. Possible causes of failures: empty pointer, the controller is not compatible with the already specified dynamical system.
             */
            bool setController(std::shared_ptr<Controller> controller);

            /**
             * @brief Returns a pointer to a DynamicalSystem object.
             * The DynamicalSystem pointer can be used inside an Integrator.
             * @return A pointer to a DynamicalSystem object.
             */
            std::shared_ptr<DynamicalSystem> asDynamicalSystem();

        private:
            class ControlledDynamicalSystemImplementation;
            ControlledDynamicalSystemImplementation* m_pimpl;
        };
    }
}

#endif // IDYNTREE_OPTIMALCONTROL_CONTROLLEDDYNAMICALSYSTEM_H
