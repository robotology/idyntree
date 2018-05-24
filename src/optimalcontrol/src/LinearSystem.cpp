/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/LinearSystem.h>
#include <iDynTree/Controller.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Utils.h>

#include <cassert>
#include <cstddef>

namespace iDynTree {
    namespace optimalcontrol {

        class LinearSystem::LinearSystemPimpl
        {
        public:
            bool timeVarying;
            size_t stateSpaceSize;
            size_t controlSpaceSize;

            iDynTree::MatrixDynSize stateMatrix;
            iDynTree::MatrixDynSize controlMatrix;

            Controller* controllerPointer;
            iDynTree::VectorDynSize controlOutput;

            LinearSystemPimpl()
                :controllerPointer(nullptr)
            {}

            ~LinearSystemPimpl(){}
        };


        LinearSystem::LinearSystem(size_t stateSize,
                                   size_t controlSize,
                                   bool isTimeVarying)
        : DynamicalSystem(stateSize, controlSize)
        , m_pimpl(new LinearSystemPimpl())
        {
            assert(m_pimpl);
            m_pimpl->controlOutput.resize(controlSize);
        }

        LinearSystem::~LinearSystem()
        {
            if(m_pimpl){
                delete m_pimpl;
                m_pimpl = nullptr;
            }
        }

        bool LinearSystem::isTimeVarying()
        {
            assert(m_pimpl);
            return m_pimpl->timeVarying;
        }

        iDynTree::MatrixDynSize& LinearSystem::stateMatrix(double time) const
        {
            assert(m_pimpl);
            //if (!m_pimpl->timeVarying) {
                return m_pimpl->stateMatrix;
            //}

        }

        iDynTree::MatrixDynSize& LinearSystem::controlMatrix(double time) const
        {
            assert(m_pimpl);
            //if (!m_pimpl->timeVarying) {
                return m_pimpl->controlMatrix;
            //}
        }


        bool LinearSystem::dynamics(const VectorDynSize& state,
                                    double time,
                                    VectorDynSize& stateDynamics)
        {
            if(!m_pimpl->controllerPointer){
                reportError("LinearSystem", "dynamics", "Controller not set.");
                return false;
            }
            if(!m_pimpl->controllerPointer->setStateFeedback(time, state)){
                reportError("LinearSystem", "dynamics", "Error while setting the feedback to the controller.");
                return false;
            }
            if(!m_pimpl->controllerPointer->doControl(m_pimpl->controlOutput)){
                reportError("LinearSystem", "dynamics", "Error while retrieving the control input.");
                return false;
            }
            iDynTree::toEigen(stateDynamics) = iDynTree::toEigen(stateMatrix(time)) * iDynTree::toEigen(state) + iDynTree::toEigen(controlMatrix(time)) * iDynTree::toEigen(m_pimpl->controlOutput);
            return true;
        }

        bool LinearSystem::dynamicsStateFirstDerivative(const VectorDynSize& state,
                                                  double time,
                                                  MatrixDynSize& dynamicsDerivative)
        {
            dynamicsDerivative = stateMatrix(time);
            return true;
        }

        bool LinearSystem::dynamicsControlFirstDerivative(const VectorDynSize& state,
                                                    double time,
                                                    MatrixDynSize& dynamicsDerivative)
        {
            dynamicsDerivative = controlMatrix(time);
            return true;
        }

    }
}
