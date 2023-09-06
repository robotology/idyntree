// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Controller.h>
#include <iDynTree/Utils.h>

#include <cstddef>


namespace iDynTree {
    namespace optimalcontrol {

        DynamicalSystem::DynamicalSystem(size_t stateSpaceSize,
                                         size_t controlSpaceSize)
        : m_stateSize(stateSpaceSize)
        , m_controlSize(controlSpaceSize)
        , m_initialState(static_cast<unsigned int>(stateSpaceSize))
        , m_controlInput(static_cast<unsigned int>(controlSpaceSize))
        {
            m_initialState.zero();
            m_controlInput.zero();
        }

        DynamicalSystem::~DynamicalSystem() {}

        size_t DynamicalSystem::stateSpaceSize() const{ return m_stateSize; }
        size_t DynamicalSystem::controlSpaceSize() const{ return m_controlSize; }

        bool DynamicalSystem::setControlInput(const VectorDynSize &control)
        {
            if (control.size() != controlSpaceSize()) {
                reportError("DynamicalSystem", "setControlInput", "Wrong control dimension.");
                return false;
            }
            m_controlInput = control;
            return true;
        }

        const VectorDynSize &DynamicalSystem::controlInput() const
        {
            return m_controlInput;
        }

        double DynamicalSystem::controlInput(unsigned int index) const
        {
            return m_controlInput(index);
        }

        const VectorDynSize &DynamicalSystem::initialState() const
        {
            return m_initialState;
        }

        double DynamicalSystem::initialState(unsigned int index) const
        {
            return m_initialState(index);
        }

        bool DynamicalSystem::setInitialState(const VectorDynSize &state)
        {
            if (state.size() != m_initialState.size()){
                reportError("DynamicalSystem", "setInitialState", "Wrong initial state dimension.");
                return false;
            }
            m_initialState = state;
            return true;
        }

        bool  DynamicalSystem::dynamicsStateFirstDerivative(const VectorDynSize& /*state*/,
                                                            double /*time*/,
                                                            MatrixDynSize& /*dynamicsDerivative*/)
        { return false; }

        bool DynamicalSystem::dynamicsControlFirstDerivative(const VectorDynSize& /*state*/,
                                                             double /*time*/,
                                                             MatrixDynSize& /*dynamicsDerivative*/)
        { return false; }

        bool DynamicalSystem::dynamicsStateFirstDerivativeSparsity(SparsityStructure &/*stateSparsity*/)
        {
            return false;
        }

        bool DynamicalSystem::dynamicsControlFirstDerivativeSparsity(SparsityStructure &/*controlSparsity*/)
        {
            return false;
        }

        bool DynamicalSystem::dynamicsSecondPartialDerivativeWRTState(double /*time*/, const VectorDynSize &/*state*/, const iDynTree::VectorDynSize &/*lambda*/, iDynTree::MatrixDynSize &/*partialDerivative*/)
        {
            return false;
        }

        bool DynamicalSystem::dynamicsSecondPartialDerivativeWRTControl(double /*time*/, const VectorDynSize &/*state*/, const iDynTree::VectorDynSize &/*lambda*/, iDynTree::MatrixDynSize &/*partialDerivative*/)
        {
            return false;
        }

        bool DynamicalSystem::dynamicsSecondPartialDerivativeWRTStateControl(double /*time*/, const VectorDynSize &/*state*/, const iDynTree::VectorDynSize &/*lambda*/, iDynTree::MatrixDynSize &/*partialDerivative*/)
        {
            return false;
        }

        bool DynamicalSystem::dynamicsSecondPartialDerivativeWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure &/*stateSparsity*/)
        {
            return false;
        }

        bool DynamicalSystem::dynamicsSecondPartialDerivativeWRTStateControlSparsity(SparsityStructure &/*stateControlSparsity*/)
        {
            return false;
        }

        bool DynamicalSystem::dynamicsSecondPartialDerivativeWRTControlSparsity(SparsityStructure &/*controlSparsity*/)
        {
            return false;
        }



    }
}
