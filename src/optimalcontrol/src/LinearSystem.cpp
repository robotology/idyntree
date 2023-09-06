// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/LinearSystem.h>

#include <iDynTree/VectorDynSize.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Utils.h>

#include <cassert>
#include <cstddef>

namespace iDynTree {
    namespace optimalcontrol {

        class LinearSystem::LinearSystemPimpl
        {
        public:
            std::shared_ptr<TimeVaryingMatrix> stateMatrix;
            std::shared_ptr<TimeVaryingMatrix> controlMatrix;

            SparsityStructure stateSparsity, controlSparsity;
            bool hasStateSparsity, hasControlSparsity;
        };


        LinearSystem::LinearSystem(size_t stateSize,
                                   size_t controlSize)
        : DynamicalSystem(stateSize, controlSize)
        , m_pimpl(new LinearSystemPimpl())
        {
            assert(m_pimpl);
            m_pimpl->hasStateSparsity = false;
            m_pimpl->hasControlSparsity = false;
            std::shared_ptr<TimeInvariantMatrix> tempState = std::make_shared<TimeInvariantMatrix>();
            tempState->get().resize(static_cast<unsigned int>(stateSize), static_cast<unsigned int>(stateSize));
            tempState->get().zero();
            m_pimpl->stateMatrix = tempState;

            std::shared_ptr<TimeInvariantMatrix> tempControl = std::make_shared<TimeInvariantMatrix>();
            tempControl->get().resize(static_cast<unsigned int>(stateSize), static_cast<unsigned int>(controlSize));
            tempControl->get().zero();
            m_pimpl->controlMatrix = tempControl;
        }

        LinearSystem::LinearSystem(size_t stateSize, size_t controlSize, const SparsityStructure &stateSparsity, const SparsityStructure &controlSparsity)
            : DynamicalSystem(stateSize, controlSize)
            , m_pimpl(new LinearSystemPimpl())
        {
            assert(m_pimpl);
            std::shared_ptr<TimeInvariantMatrix> tempState = std::make_shared<TimeInvariantMatrix>();
            tempState->get().resize(static_cast<unsigned int>(stateSize), static_cast<unsigned int>(stateSize));
            tempState->get().zero();
            m_pimpl->stateMatrix = tempState;

            std::shared_ptr<TimeInvariantMatrix> tempControl = std::make_shared<TimeInvariantMatrix>();
            tempControl->get().resize(static_cast<unsigned int>(stateSize), static_cast<unsigned int>(controlSize));
            tempControl->get().zero();
            m_pimpl->controlMatrix = tempControl;

            m_pimpl->hasStateSparsity = false;
            m_pimpl->hasControlSparsity = false;
            if (stateSparsity.isValid()) {
                m_pimpl->hasStateSparsity = true;
                m_pimpl->stateSparsity = stateSparsity;
            }
            if (controlSparsity.isValid()) {
                m_pimpl->hasControlSparsity = true;
                m_pimpl->controlSparsity = controlSparsity;
            }
        }

        LinearSystem::~LinearSystem()
        {
            if(m_pimpl){
                delete m_pimpl;
                m_pimpl = nullptr;
            }
        }

        bool LinearSystem::setStateMatrix(const MatrixDynSize &stateMatrix)
        {
            if ((stateMatrix.rows() != stateSpaceSize()) || (stateMatrix.cols() != stateSpaceSize())) {
                reportError("LinearSystem", "setStateMatrix", "The stateMatrix does not match the specified state space size.");
                return false;
            }
            m_pimpl->stateMatrix.reset(new TimeInvariantMatrix(stateMatrix));
            return true;
        }

        bool LinearSystem::setControlMatrix(const MatrixDynSize &controlMatrix)
        {
            if ((controlMatrix.rows() != stateSpaceSize()) || (controlMatrix.cols() != controlSpaceSize())) {
                reportError("LinearSystem", "setControlMatrix", "The controlMatrix does not match the specified state/control space size.");
                return false;
            }
            m_pimpl->controlMatrix.reset(new TimeInvariantMatrix(controlMatrix));
            return true;
        }

        bool LinearSystem::setStateMatrix(std::shared_ptr<TimeVaryingMatrix> stateMatrix)
        {
            if (!stateMatrix) {
                reportError("LinearSystem", "setStateMatrix", "Empty stateMatrix pointer.");
                return false;
            }
            m_pimpl->stateMatrix = stateMatrix;
            return true;
        }

        bool LinearSystem::setControlMatrix(std::shared_ptr<TimeVaryingMatrix> controlMatrix)
        {
            if (!controlMatrix) {
                reportError("LinearSystem", "setControlMatrix", "Empty controlMatrix pointer.");
                return false;
            }
            m_pimpl->controlMatrix = controlMatrix;
            return true;
        }


        bool LinearSystem::dynamics(const VectorDynSize& state,
                                    double time,
                                    VectorDynSize& stateDynamics)
        {
            bool isValid = false;
            const MatrixDynSize& stateMatrix = m_pimpl->stateMatrix->get(time, isValid);

            if (!isValid) {
                std::ostringstream errorMsg;
                errorMsg << "Unable to retrieve a valid state matrix at time: " << time << ".";
                reportError("LinearSystem", "dynamics", errorMsg.str().c_str());
                return false;
            }

            if ((stateMatrix.rows() != stateSpaceSize()) || (stateMatrix.cols() != stateSpaceSize())) {
                std::ostringstream errorMsg;
                errorMsg << "The state matrix at time: " << time << " has dimensions not matching with the specified state space dimension.";
                reportError("LinearSystem", "dynamics", errorMsg.str().c_str());
                return false;
            }

            isValid = false;
            const MatrixDynSize& controlMatrix = m_pimpl->controlMatrix->get(time, isValid);

            if (!isValid) {
                std::ostringstream errorMsg;
                errorMsg << "Unable to retrieve a valid control matrix at time: " << time << ".";
                reportError("LinearSystem", "dynamics", errorMsg.str().c_str());
                return false;
            }

            if ((controlMatrix.rows() != stateSpaceSize()) || (controlMatrix.cols() != controlSpaceSize())) {
                std::ostringstream errorMsg;
                errorMsg << "The control matrix at time: " << time << " has dimensions not matching with the specified state/control space dimension.";
                reportError("LinearSystem", "dynamics", errorMsg.str().c_str());
                return false;
            }

            iDynTree::toEigen(stateDynamics) = iDynTree::toEigen(stateMatrix) * iDynTree::toEigen(state) + iDynTree::toEigen(controlMatrix) * iDynTree::toEigen(controlInput());
            return true;
        }

        bool LinearSystem::dynamicsStateFirstDerivative(const VectorDynSize& /*state*/,
                                                  double time,
                                                  MatrixDynSize& dynamicsDerivative)
        {
            bool isValid = false;
            const MatrixDynSize& stateMatrix = m_pimpl->stateMatrix->get(time, isValid);

            if (!isValid) {
                std::ostringstream errorMsg;
                errorMsg << "Unable to retrieve a valid state matrix at time: " << time << ".";
                reportError("LinearSystem", "dynamicsStateFirstDerivative", errorMsg.str().c_str());
                return false;
            }

            if ((stateMatrix.rows() != stateSpaceSize()) || (stateMatrix.cols() != stateSpaceSize())) {
                std::ostringstream errorMsg;
                errorMsg << "The state matrix at time: " << time << " has dimensions not matching with the specified state space dimension.";
                reportError("LinearSystem", "dynamicsStateFirstDerivative", errorMsg.str().c_str());
                return false;
            }

            dynamicsDerivative = stateMatrix;
            return true;
        }

        bool LinearSystem::dynamicsControlFirstDerivative(const VectorDynSize& /*state*/,
                                                    double time,
                                                    MatrixDynSize& dynamicsDerivative)
        {
            bool isValid = false;
            const MatrixDynSize& controlMatrix = m_pimpl->controlMatrix->get(time, isValid);

            if (!isValid) {
                std::ostringstream errorMsg;
                errorMsg << "Unable to retrieve a valid control matrix at time: " << time << ".";
                reportError("LinearSystem", "dynamicsControlFirstDerivative", errorMsg.str().c_str());
                return false;
            }

            if ((controlMatrix.rows() != stateSpaceSize()) || (controlMatrix.cols() != controlSpaceSize())) {
                std::ostringstream errorMsg;
                errorMsg << "The control matrix at time: " << time << " has dimensions not matching with the specified state/control space dimension.";
                reportError("LinearSystem", "dynamicsControlFirstDerivative", errorMsg.str().c_str());
                return false;
            }

            dynamicsDerivative = controlMatrix;
            return true;
        }

        bool LinearSystem::dynamicsStateFirstDerivativeSparsity(SparsityStructure &stateSparsity)
        {
            if (!(m_pimpl->hasStateSparsity)) {
                return false;
            }
            stateSparsity = m_pimpl->stateSparsity;
            return true;
        }

        bool LinearSystem::dynamicsControlFirstDerivativeSparsity(SparsityStructure &controlSparsity)
        {
            if (!(m_pimpl->hasControlSparsity)) {
                return false;
            }
            controlSparsity = m_pimpl->controlSparsity;
            return true;
        }

        bool LinearSystem::dynamicsSecondPartialDerivativeWRTState(double /*time*/, const VectorDynSize &state, const VectorDynSize &/*lambda*/, MatrixDynSize &partialDerivative)
        {
            partialDerivative.resize(state.size(), state.size());
            partialDerivative.zero();
            return true;
        }

        bool LinearSystem::dynamicsSecondPartialDerivativeWRTControl(double /*time*/, const VectorDynSize &/*state*/, const VectorDynSize &/*lambda*/, MatrixDynSize &partialDerivative)
        {
            partialDerivative.resize(controlInput().size(), controlInput().size());
            partialDerivative.zero();
            return true;
        }

        bool LinearSystem::dynamicsSecondPartialDerivativeWRTStateControl(double /*time*/, const VectorDynSize &state, const VectorDynSize &/*lambda*/, MatrixDynSize &partialDerivative)
        {
            partialDerivative.resize(state.size(), controlInput().size());
            partialDerivative.zero();
            return true;
        }

        bool LinearSystem::dynamicsSecondPartialDerivativeWRTStateSparsity(SparsityStructure &stateSparsity)
        {
            stateSparsity.clear();
            return true;
        }

        bool LinearSystem::dynamicsSecondPartialDerivativeWRTStateControlSparsity(SparsityStructure &stateControlSparsity)
        {
            stateControlSparsity.clear();
            return true;
        }

        bool LinearSystem::dynamicsSecondPartialDerivativeWRTControlSparsity(SparsityStructure &controlSparsity)
        {
            controlSparsity.clear();
            return true;
        }

    }
}
