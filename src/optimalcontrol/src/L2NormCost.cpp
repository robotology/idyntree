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

        class TimeVaryingGradient : public TimeVaryingVector {
            MatrixDynSize m_selectorMatrix, m_weightMatrix;
            MatrixDynSize m_subMatrix; //weightMatrix times selector
            std::shared_ptr<TimeVaryingVector> m_desiredTrajectory;
            VectorDynSize m_outputVector;
        public:
            TimeVaryingGradient(const MatrixDynSize& selectorMatrix)
            : m_selectorMatrix(selectorMatrix)
            , m_desiredTrajectory(nullptr)
            {
                m_weightMatrix.resize(m_selectorMatrix.rows(), m_selectorMatrix.rows());
                toEigen(m_weightMatrix).setIdentity();
                m_subMatrix = /*m_pimpl->stateWeight * */ m_selectorMatrix;
                m_outputVector.resize(m_selectorMatrix.cols());
                m_outputVector.zero();
            }

            ~TimeVaryingGradient() override;

            bool setDesiredTrajectory(std::shared_ptr<TimeVaryingVector> desiredTrajectory) {
                if (!desiredTrajectory) {
                    reportError("TimeVaryingGradient", "desiredTrajectory", "Empty desired trajectory pointer.");
                    return false;
                }
                m_desiredTrajectory = desiredTrajectory;
                return true;
            }

            bool setWeightMatrix(const MatrixDynSize &weights)
            {
                if (weights.rows() != weights.cols()) {
                    reportError("TimeVaryingGradient", "setWeightMatrix", "The weights matrix is supposed to be squared.");
                    return false;
                }

                if (weights.cols() != m_selectorMatrix.rows()) {
                    reportError("TimeVaryingGradient", "setWeightMatrix", "The weights matrix dimensions do not match those of the specified selector.");
                    return false;
                }

                m_weightMatrix = weights;
                toEigen(m_subMatrix) = toEigen(m_weightMatrix) * toEigen(m_selectorMatrix);

                return true;
            }

            virtual const VectorDynSize& get(double time, bool &isValid) override {
                if (!m_desiredTrajectory) {
                    isValid = true;
                    return m_outputVector; //should be zero from the initialization
                }
                bool ok = false;
                const VectorDynSize &desiredPoint = m_desiredTrajectory->get(time, ok);

                if (!ok) {
                    isValid = false;
                    m_outputVector.zero();
                    return m_outputVector;
                }

                if (desiredPoint.size() != m_subMatrix.rows()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The specified desired point at time: " << time << " has size not matching the specified selector.";
                    reportError("TimeVaryingGradient", "getObject", errorMsg.str().c_str());
                    isValid = false;
                    m_outputVector.zero();
                    return m_outputVector;
                }

                toEigen(m_outputVector) = -1.0 * toEigen(desiredPoint).transpose() * toEigen(m_subMatrix);

                isValid = true;
                return m_outputVector;
            }

            bool updateSelector(const MatrixDynSize& selector) {
                if ((selector.rows() != m_selectorMatrix.rows()) || (selector.cols() != m_selectorMatrix.cols())) {
                    reportError("TimeVaryingGradient", "getObject", "The new selector dimensionsions do not match the old one.");
                    return false;
                }
                m_selectorMatrix = selector;
                toEigen(m_subMatrix) = toEigen(m_weightMatrix) * toEigen(m_selectorMatrix);

                return true;
            }

            const MatrixDynSize& selector() {
                return m_selectorMatrix;
            }

            const MatrixDynSize& subMatrix() {
                return m_subMatrix;
            }

            const MatrixDynSize& weightMatrix() {
                return m_weightMatrix;
            }

            std::shared_ptr<TimeVaryingVector> desiredTrajectory() {
                return m_desiredTrajectory;
            }
        };
        TimeVaryingGradient::~TimeVaryingGradient() {};

        class TimeVaryingBias : public TimeVaryingDouble {
            std::shared_ptr<TimeVaryingGradient> m_associatedGradient;
            double m_output;
        public:
            TimeVaryingBias(std::shared_ptr<TimeVaryingGradient> associatedGradient)
            :m_associatedGradient(associatedGradient)
            { }

            ~TimeVaryingBias() override;

            virtual const double& get(double time, bool &isValid) override{
                if (!(m_associatedGradient)) {
                    isValid = false;
                    m_output = 0.0;
                    return m_output;
                }

                if (!(m_associatedGradient->desiredTrajectory())) {
                    isValid = true;
                    m_output = 0.0;
                    return m_output;
                }

                bool ok = false;
                const VectorDynSize &desiredPoint = m_associatedGradient->desiredTrajectory()->get(time, ok);

                if (!ok) {
                    isValid = false;
                    m_output = 0.0;
                    return m_output;
                }

                if (desiredPoint.size() != m_associatedGradient->weightMatrix().rows()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The specified desired point at time: " << time << " has size not matching the specified weight matrix.";
                    reportError("TimeVaryingBias", "getObject", errorMsg.str().c_str());
                    isValid = false;
                    m_output = 0.0;
                    return m_output;
                }

                isValid = true;
                m_output = 0.5 * toEigen(desiredPoint).transpose() * toEigen(m_associatedGradient->weightMatrix()) * toEigen(desiredPoint);
                return m_output;
            }
        };
        TimeVaryingBias::~TimeVaryingBias() { }


        class L2NormCost::L2NormCostImplementation {
            public:
            std::shared_ptr<TimeVaryingGradient> stateGradient = nullptr, controlGradient = nullptr;
            std::shared_ptr<TimeInvariantMatrix> stateHessian, controlHessian;
            std::shared_ptr<TimeVaryingBias> stateCostBias, controlCostBias;
            bool addConstantPart = false;

            void initialize(const MatrixDynSize &stateSelector, const MatrixDynSize &controlSelector) {
                if ((stateSelector.rows() != 0) && (stateSelector.cols() != 0)) {
                    stateGradient = std::make_shared<TimeVaryingGradient>(stateSelector);
                    stateHessian = std::make_shared<TimeInvariantMatrix>();
                    stateHessian->get().resize(stateSelector.cols(), stateSelector.cols());
                    toEigen(stateHessian->get()) = toEigen(stateGradient->selector()).transpose() * toEigen(stateGradient->subMatrix());
                    stateCostBias = std::make_shared<TimeVaryingBias>(stateGradient);
                } else {
                    stateGradient = nullptr;
                }

                if ((controlSelector.rows() != 0) && (controlSelector.cols() != 0)) {
                    controlGradient = std::make_shared<TimeVaryingGradient>(controlSelector);
                    controlHessian = std::make_shared<TimeInvariantMatrix>();
                    controlHessian->get().resize(controlSelector.cols(), controlSelector.cols());
                    toEigen(controlHessian->get()) = toEigen(controlGradient->selector()).transpose() * toEigen(controlGradient->subMatrix());
                    controlCostBias = std::make_shared<TimeVaryingBias>(controlGradient);
                } else {
                    controlGradient = nullptr;
                }
            }

            void initialize(unsigned int stateDimension, unsigned int controlDimension) {
                MatrixDynSize stateSelector(stateDimension, stateDimension), controlSelector(controlDimension, controlDimension);
                toEigen(stateSelector).setIdentity();
                toEigen(controlSelector).setIdentity();
                initialize(stateSelector, controlSelector);
            }

        };


        L2NormCost::L2NormCost(const std::string &name, unsigned int stateDimension, unsigned int controlDimension)
        : QuadraticLikeCost(name)
        , m_pimpl(new L2NormCostImplementation)
        {
            assert(m_pimpl);

            m_pimpl->initialize(stateDimension, controlDimension);

            if (m_pimpl->stateGradient) {
                m_timeVaryingStateHessian = m_pimpl->stateHessian;
                m_timeVaryingStateGradient = m_pimpl->stateGradient;
            }

            if (m_pimpl->controlGradient) {
                m_timeVaryingControlHessian = m_pimpl->controlHessian;
                m_timeVaryingControlGradient = m_pimpl->controlGradient;
            }
        }

        L2NormCost::L2NormCost(const std::string &name, const MatrixDynSize &stateSelector, const MatrixDynSize &controlSelector)
        : QuadraticLikeCost(name)
        , m_pimpl(new L2NormCostImplementation)
        {
            m_pimpl->initialize(stateSelector, controlSelector);

            if (m_pimpl->stateGradient) {
                m_timeVaryingStateHessian = m_pimpl->stateHessian;
                m_timeVaryingStateGradient = m_pimpl->stateGradient;
            }

            if (m_pimpl->controlGradient) {
                m_timeVaryingControlHessian = m_pimpl->controlHessian;
                m_timeVaryingControlGradient = m_pimpl->controlGradient;
            }
        }

        L2NormCost::~L2NormCost()
        {
            if (m_pimpl){
                delete m_pimpl;
                m_pimpl = nullptr;
            }
        }

        void L2NormCost::computeConstantPart(bool addItToTheCost)
        {
            m_pimpl->addConstantPart = addItToTheCost;
            if (m_pimpl->addConstantPart) {
                if (m_pimpl->stateGradient) {
                    m_timeVaryingStateCostBias = m_pimpl->stateCostBias;
                }
                if (m_pimpl->controlGradient) {
                    m_timeVaryingControlCostBias = m_pimpl->controlCostBias;
                }
            } else {
                std::shared_ptr<TimeVaryingDouble> stateBias(new TimeInvariantDouble(0.0)), controlBias(new TimeInvariantDouble(0.0));
                if (m_pimpl->stateGradient) {
                    m_timeVaryingStateCostBias = stateBias;
                }
                if (m_pimpl->controlGradient) {
                    m_timeVaryingControlCostBias = controlBias;
                }
            }
        }

        bool L2NormCost::setStateWeight(const MatrixDynSize &stateWeights)
        {
            if (stateWeights.rows() != stateWeights.cols()) {
                reportError("L2NormCost", "setStateWeight", "The stateWeights matrix is supposed to be squared.");
                return false;
            }

            if (!(m_pimpl->stateGradient)) {
                reportError("L2NormCost", "setStateWeight", "The state cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (!(m_pimpl->stateGradient->setWeightMatrix(stateWeights))) {
                reportError("L2NormCost", "setStateWeight", "Error when specifying the state weights.");
                return false;
            }

            return true;
        }

        bool L2NormCost::setStateDesiredPoint(const VectorDynSize &desiredPoint)
        {
            if (!(m_pimpl->stateGradient)) {
                reportError("L2NormCost", "setStateDesiredPoint", "The state cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (desiredPoint.size() != (m_pimpl->stateGradient->subMatrix().rows())) {
                reportError("L2NormCost", "setStateDesiredPoint", "The desiredPoint size do not match the dimension of the specified selector.");
                return false;
            }

            std::shared_ptr<TimeInvariantVector> newTrajectory = std::make_shared<TimeInvariantVector>(desiredPoint);
            return m_pimpl->stateGradient->setDesiredTrajectory(newTrajectory);
        }

        bool L2NormCost::setStateDesiredTrajectory(std::shared_ptr<TimeVaryingVector> stateDesiredTrajectory)
        {
            if (!(m_pimpl->stateGradient)) {
                reportError("L2NormCost", "setStateDesiredTrajectory", "The state cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (!stateDesiredTrajectory) {
                reportError("L2NormCost", "setStateDesiredTrajectory", "Empty desired trajectory pointer.");
                return false;
            }

            return m_pimpl->stateGradient->setDesiredTrajectory(stateDesiredTrajectory);
        }

        bool L2NormCost::setControlWeight(const MatrixDynSize &controlWeights)
        {
            if (controlWeights.rows() != controlWeights.cols()) {
                reportError("L2NormCost", "setControlWeight", "The controlWeights matrix is supposed to be squared.");
                return false;
            }

            if (!(m_pimpl->controlGradient)) {
                reportError("L2NormCost", "setControlWeight", "The control cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (!(m_pimpl->controlGradient->setWeightMatrix(controlWeights))) {
                reportError("L2NormCost", "setControlWeight", "Error when specifying the control weights.");
                return false;
            }

            return true;
        }

        bool L2NormCost::setControlDesiredPoint(const VectorDynSize &desiredPoint)
        {
            if (!(m_pimpl->controlGradient)) {
                reportError("L2NormCost", "setControlDesiredPoint", "The control cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (desiredPoint.size() != (m_pimpl->controlGradient->subMatrix().rows())) {
                reportError("L2NormCost", "setControlDesiredPoint", "The desiredPoint size do not match the dimension of the specified selector.");
                return false;
            }

            std::shared_ptr<TimeInvariantVector> newTrajectory = std::make_shared<TimeInvariantVector>(desiredPoint);
            return m_pimpl->controlGradient->setDesiredTrajectory(newTrajectory);
        }

        bool L2NormCost::setControlDesiredTrajectory(std::shared_ptr<TimeVaryingVector> controlDesiredTrajectory)
        {
            if (!(m_pimpl->controlGradient)) {
                reportError("L2NormCost", "setControlDesiredTrajectory", "The control cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (!controlDesiredTrajectory) {
                reportError("L2NormCost", "setControlDesiredTrajectory", "Empty desired trajectory pointer.");
                return false;
            }

            return m_pimpl->controlGradient->setDesiredTrajectory(controlDesiredTrajectory);
        }

        bool L2NormCost::updatStateSelector(const MatrixDynSize &stateSelector)
        {
            if (!(m_pimpl->stateGradient->updateSelector(stateSelector))) {
                reportError("L2NormCost", "updatStateSelector", "Error when updating state selector.");
                return false;
            }
            toEigen(m_pimpl->stateHessian->get()) = toEigen(m_pimpl->stateGradient->selector()).transpose() * toEigen(m_pimpl->stateGradient->subMatrix());
            return true;
        }

        bool L2NormCost::updatControlSelector(const MatrixDynSize &controlSelector)
        {
            if (!(m_pimpl->controlGradient->updateSelector(controlSelector))) {
                reportError("L2NormCost", "updatControlSelector", "Error when updating state selector.");
                return false;
            }
            toEigen(m_pimpl->controlHessian->get()) = toEigen(m_pimpl->controlGradient->selector()).transpose() * toEigen(m_pimpl->controlGradient->subMatrix());
            return true;
        }

    }
}

