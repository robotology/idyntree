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

#include <iDynTree/L2NormCost.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Utils.h>
#include <cassert>

#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree {
    namespace optimalcontrol {

        class L2NormCost::L2NormCostImplementation {
            public:
            bool stateSelectorDefined = false, controlSelectorDefined = false;
            MatrixDynSize stateSelector, controlSelector;
            MatrixDynSize stateWeight, controlWeight;
            MatrixDynSize stateSubMatrix, controlSubMatrix;
            bool desiredStateDefined = false, desiredControlDefined = false;
            VectorDynSize desiredState, desiredControl;
            VectorDynSize stateGradient, controlGradient;
            MatrixDynSize stateHessian, controlHessian;
        };

        L2NormCost::L2NormCost(const std::string &name)
        : QuadraticCost(name)
        , m_pimpl(new L2NormCostImplementation)
        {
            assert(m_pimpl);
        }

        L2NormCost::L2NormCost(const std::string &name, const MatrixDynSize &stateSelector)
        : QuadraticCost(name)
        , m_pimpl(new L2NormCostImplementation)
        {
            assert(m_pimpl);
            m_pimpl->stateSelector = stateSelector;
            m_pimpl->stateSelectorDefined = true;
            m_pimpl->stateWeight.resize(stateSelector.rows(), stateSelector.rows());
            toEigen(m_pimpl->stateWeight).setIdentity();
            m_pimpl->stateSubMatrix = /*m_pimpl->stateWeight * */ stateSelector;
            m_pimpl->desiredState.resize(stateSelector.rows());
            m_pimpl->desiredState.zero();
            m_pimpl->stateGradient.resize(stateSelector.cols());
            m_pimpl->stateGradient.zero();
            m_pimpl->stateHessian.resize(stateSelector.cols(), stateSelector.cols());
            toEigen(m_pimpl->stateHessian) = toEigen(m_pimpl->stateSelector).transpose()*toEigen(m_pimpl->stateSubMatrix);
        }

        L2NormCost::L2NormCost(const std::string &name, const MatrixDynSize &stateSelector, const MatrixDynSize &controlSelector)
        : QuadraticCost(name)
        , m_pimpl(new L2NormCostImplementation)
        {
            assert(m_pimpl);
            m_pimpl->stateSelector = stateSelector;
            m_pimpl->stateSelectorDefined = true;
            m_pimpl->stateWeight.resize(stateSelector.rows(), stateSelector.rows());
            toEigen(m_pimpl->stateWeight).setIdentity();
            m_pimpl->stateSubMatrix = /*m_pimpl->stateWeight * */ stateSelector;
            m_pimpl->desiredState.resize(stateSelector.rows());
            m_pimpl->desiredState.zero();
            m_pimpl->stateGradient.resize(stateSelector.cols());
            m_pimpl->stateGradient.zero();
            m_pimpl->stateHessian.resize(stateSelector.cols(), stateSelector.cols());
            toEigen(m_pimpl->stateHessian) = toEigen(m_pimpl->stateSelector).transpose()*toEigen(m_pimpl->stateSubMatrix);

            m_pimpl->controlSelector = controlSelector;
            m_pimpl->controlSelectorDefined = true;
            m_pimpl->controlWeight.resize(controlSelector.rows(), controlSelector.rows());
            toEigen(m_pimpl->controlWeight).setIdentity();
            m_pimpl->controlSubMatrix = /*m_pimpl->controlWeight * */ controlSelector;
            m_pimpl->desiredControl.resize(controlSelector.rows());
            m_pimpl->desiredControl.zero();
            m_pimpl->controlGradient.resize(controlSelector.cols());
            m_pimpl->controlGradient.zero();
            m_pimpl->controlHessian.resize(controlSelector.cols(), controlSelector.cols());
            toEigen(m_pimpl->controlHessian) = toEigen(m_pimpl->controlSelector).transpose()*toEigen(m_pimpl->controlSubMatrix);
        }

        L2NormCost::~L2NormCost()
        {
            if (m_pimpl){
                delete m_pimpl;
                m_pimpl = nullptr;
            }
        }

        bool L2NormCost::setStateWeight(const VectorDynSize &stateWeights)
        {

        }

        bool L2NormCost::setStateWeight(const MatrixDynSize &stateWeights)
        {
            if (stateWeights.rows() != stateWeights.cols()) {
                reportError("L2NormCost", "setStateWeight", "The stateWeights matrix is supposed to be squared.");
                return false;
            }

            if (m_pimpl->stateSelectorDefined) {

                if (stateWeights.cols() != m_pimpl->stateSelector.rows()) {
                    reportError("L2NormCost", "setStateWeight", "The stateWeights matrix dimensions do not match those of the specified state selector.");
                    return false;
                }

                m_pimpl->stateWeight = stateWeights;
                toEigen(m_pimpl->stateSubMatrix) = toEigen(stateWeights) * toEigen(m_pimpl->stateSelector);
                toEigen(m_pimpl->stateGradient) = -1.0 * toEigen(m_pimpl->desiredState).transpose() * toEigen(m_pimpl->stateSubMatrix);
                toEigen(m_pimpl->stateHessian) = toEigen(m_pimpl->stateSelector).transpose()*toEigen(m_pimpl->stateSubMatrix);

                return setStateCost(m_pimpl->stateHessian, m_pimpl->stateGradient);
            } else {

                m_pimpl->stateWeight = stateWeights;
                m_pimpl->stateSubMatrix = stateWeights;
                m_pimpl->stateHessian = stateWeights;

                if (m_pimpl->desiredStateDefined) {
                    toEigen(m_pimpl->stateGradient) = -1.0 * toEigen(m_pimpl->desiredState).transpose() * toEigen(m_pimpl->stateSubMatrix);
                } else {
                    m_pimpl->stateGradient.resize(stateWeights.rows());
                }

                return setStateCost(m_pimpl->stateHessian, m_pimpl->stateGradient);
            }

        }


    }
}

