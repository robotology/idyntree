// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/ControlledDynamicalSystem.h>
#include <iDynTree/Utils.h>
#include <cassert>

namespace iDynTree {
    namespace optimalcontrol {

        class DerivedDynamicalSystem : public DynamicalSystem {
            std::shared_ptr<DynamicalSystem> m_autonomousSystem;
            std::shared_ptr<Controller> m_controller;
            VectorDynSize m_controlBuffer, m_empty;
        public:

            DerivedDynamicalSystem()
            : DynamicalSystem(0,0)
            , m_autonomousSystem(nullptr)
            , m_controller(nullptr)
            {}

            DerivedDynamicalSystem(std::shared_ptr<DynamicalSystem> autonomousSystem)
            : DynamicalSystem(autonomousSystem->stateSpaceSize(), autonomousSystem->controlSpaceSize())
            , m_autonomousSystem(autonomousSystem)
            , m_controller(nullptr)
            , m_controlBuffer(static_cast<unsigned int>(autonomousSystem->controlSpaceSize()))
            {}

            virtual ~DerivedDynamicalSystem();

            bool setController(std::shared_ptr<Controller> controller) {

                if (!m_autonomousSystem) {
                    reportError("ControlledDynamicalSystem", "setController", "Dynamical system not set.");
                    return false;
                }

                if (!controller) {
                    reportError("ControlledDynamicalSystem", "setController", "Empty Controller pointer.");
                    return false;
                }

                if (controller->controlSpaceSize() != m_autonomousSystem->controlSpaceSize()) {
                    reportError("ControlledDynamicalSystem", "setController", "The controller size does not match the dynamical system control size.");
                    return false;
                }

                m_controller = controller;

                return true;
            }

            virtual bool dynamics(const VectorDynSize& state,
                                  double time,
                                  VectorDynSize& stateDynamics) final {
                if (!m_autonomousSystem) {
                    reportError("ControlledDynamicalSystem", "dynamics", "No DynamicalSystem set.");
                    return false;
                }

                if (!m_controller) {
                    reportError("ControlledDynamicalSystem", "dynamics", "No Controller set.");
                    return false;
                }

                if (!m_controller->setStateFeedback(time, state)) {
                    reportError("ControlledDynamicalSystem", "dynamics", "Error while setting feedback to the controller.");
                    return false;
                }

                if (!m_controller->doControl(m_controlBuffer)) {
                    reportError("ControlledDynamicalSystem", "dynamics", "Error while retrieving control input.");
                    return false;
                }

                if (!m_autonomousSystem->setControlInput(m_controlBuffer)) {
                    reportError("ControlledDynamicalSystem", "dynamics", "Error while setting the control input to the dynamical system.");
                    return false;
                }

                if (!m_autonomousSystem->dynamics(state, time, stateDynamics)) {
                    reportError("ControlledDynamicalSystem", "dynamics", "Error while retrieving the dynamics from the autonomous system.");
                    return false;
                }

                return true;
            }

            virtual bool setControlInput(const VectorDynSize &control) final {
                reportError("ControlledDynamicalSystem", "setControlInput", "Cannot set the control input to a controlled dynamical system.");
                return false;
            }

            virtual const VectorDynSize& initialState() const final {
                if (m_autonomousSystem) {
                    return m_autonomousSystem->initialState();
                }
                reportError("ControlledDynamicalSystem", "initialState", "No dynamical system set");
                return m_empty;
            }

            virtual bool setInitialState(const VectorDynSize &state) final{
                if (m_autonomousSystem) {
                    return m_autonomousSystem->setInitialState(state);
                }
                reportError("ControlledDynamicalSystem", "setInitialState", "No dynamical system set");
                return false;
            }


            virtual bool dynamicsStateFirstDerivative(const VectorDynSize& state,
                                                      double time,
                                                      MatrixDynSize& dynamicsDerivative) final {
                if (!m_autonomousSystem) {
                    reportError("ControlledDynamicalSystem", "dynamicsStateFirstDerivative", "No DynamicalSystem set.");
                    return false;
                }

                if (!m_controller) {
                    reportError("ControlledDynamicalSystem", "dynamicsStateFirstDerivative", "No Controller set.");
                    return false;
                }

                if (!m_controller->setStateFeedback(time, state)) {
                    reportError("ControlledDynamicalSystem", "dynamicsStateFirstDerivative", "Error while setting feedback to the controller.");
                    return false;
                }

                if (!m_controller->doControl(m_controlBuffer)) {
                    reportError("ControlledDynamicalSystem", "dynamicsStateFirstDerivative", "Error while retrieving control input.");
                    return false;
                }

                if (!m_autonomousSystem->setControlInput(m_controlBuffer)) {
                    reportError("ControlledDynamicalSystem", "dynamicsStateFirstDerivative", "Error while setting the control input to the dynamical system.");
                    return false;
                }

                if (!m_autonomousSystem->dynamicsStateFirstDerivative(state, time, dynamicsDerivative)) {
                    reportError("ControlledDynamicalSystem", "dynamicsStateFirstDerivative", "Error while retrieving derivative from the dynamical system.");
                    return false;
                }
                return true;
            }

            virtual bool dynamicsControlFirstDerivative(const VectorDynSize& state,
                                                        double time,
                                                        MatrixDynSize& dynamicsDerivative) final {
                if (!m_autonomousSystem) {
                    reportError("ControlledDynamicalSystem", "dynamicsControlFirstDerivative", "No DynamicalSystem set.");
                    return false;
                }

                if (!m_controller) {
                    reportError("ControlledDynamicalSystem", "dynamicsControlFirstDerivative", "No Controller set.");
                    return false;
                }

                if (!m_controller->setStateFeedback(time, state)) {
                    reportError("ControlledDynamicalSystem", "dynamicsControlFirstDerivative", "Error while setting feedback to the controller.");
                    return false;
                }

                if (!m_controller->doControl(m_controlBuffer)) {
                    reportError("ControlledDynamicalSystem", "dynamicsControlFirstDerivative", "Error while retrieving control input.");
                    return false;
                }

                if (!m_autonomousSystem->setControlInput(m_controlBuffer)) {
                    reportError("ControlledDynamicalSystem", "dynamicsControlFirstDerivative", "Error while setting the control input to the dynamical system.");
                    return false;
                }

                if (!m_autonomousSystem->dynamicsControlFirstDerivative(state, time, dynamicsDerivative)) {
                    reportError("ControlledDynamicalSystem", "dynamicsControlFirstDerivative", "Error while retrieving derivative from the dynamical system.");
                    return false;
                }
                return true;
            }
        };
        DerivedDynamicalSystem::~DerivedDynamicalSystem(){};

        class ControlledDynamicalSystem::ControlledDynamicalSystemImplementation {
        public:
            std::shared_ptr<Controller> lastController;
            std::shared_ptr<DerivedDynamicalSystem> controlledSystem;
            std::shared_ptr<DynamicalSystem> asDynamicalSystem;


            ControlledDynamicalSystemImplementation()
            :lastController(nullptr)
            ,controlledSystem(new DerivedDynamicalSystem())
            ,asDynamicalSystem(controlledSystem)
            {}

        };

        ControlledDynamicalSystem::ControlledDynamicalSystem()
        :m_pimpl(new ControlledDynamicalSystemImplementation)
        {
            assert(m_pimpl);
        }

        ControlledDynamicalSystem::~ControlledDynamicalSystem()
        {
            if (m_pimpl) {
                delete m_pimpl;
            }
            m_pimpl = nullptr;
        }

        bool ControlledDynamicalSystem::setDynamicalSystem(std::shared_ptr<DynamicalSystem> autonomousSystem, bool usePreviousController)
        {
            if (!autonomousSystem) {
                reportError("ControlledDynamicalSystem", "setDynamicalSystem", "Empty dynamical system pointer.");
                return false;
            }

            m_pimpl->controlledSystem.reset(new DerivedDynamicalSystem(autonomousSystem));

            if (usePreviousController && (m_pimpl->lastController)) {
                if (!(m_pimpl->controlledSystem->setController(m_pimpl->lastController))) {
                    return false;
                }
            }

            m_pimpl->asDynamicalSystem = m_pimpl->controlledSystem;
            return true;
        }

        bool ControlledDynamicalSystem::setController(std::shared_ptr<Controller> controller)
        {
            if (!controller) {
                reportError("ControlledDynamicalSystem", "setController", "Empty controller  pointer.");
                return false;
            }

            if (m_pimpl->controlledSystem->stateSpaceSize() != 0) {
                if (!(m_pimpl->controlledSystem->setController(m_pimpl->lastController))) {
                    return false;
                }
            }

            m_pimpl->lastController = controller;
            return true;
        }

        std::shared_ptr<DynamicalSystem> ControlledDynamicalSystem::asDynamicalSystem()
        {
            if (m_pimpl->controlledSystem->stateSpaceSize() == 0)
                reportWarning("ControlledDynamicalSystem", "asDynamicalSystem", "No dynamical system has been set yet.");

            return m_pimpl->asDynamicalSystem;
        }



    }
}
