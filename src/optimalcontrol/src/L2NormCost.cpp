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
#include <iDynTree/Core/Span.h>
#include <cassert>
#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree {
    namespace optimalcontrol {

    //
    // Implementation of selector matrix
    //
        class Selector {
        protected:
            VectorDynSize m_selected;
        public:
            Selector(unsigned int selectedSize) : m_selected(selectedSize) { }

            virtual ~Selector();

            virtual const VectorDynSize& select(const VectorDynSize& fullVector) = 0;

            unsigned int size() const {
                return m_selected.size();
            }

        };
        Selector::~Selector(){}

        class IndexSelector : public Selector {
            IndexRange m_selectedIndex;
        public:
            IndexSelector(const IndexRange& selectedRange)
                : Selector(static_cast<unsigned int>(selectedRange.size))
                , m_selectedIndex(selectedRange) {}

            ~IndexSelector() override;

            virtual const VectorDynSize& select(const VectorDynSize& fullVector) final
            {
                m_selected = make_span(fullVector).subspan(m_selectedIndex.offset, m_selectedIndex.size);
                return m_selected;
            }
        };
        IndexSelector::~IndexSelector(){}

        class MatrixSelector : public Selector {
            MatrixDynSize m_selectorMatrix;
        public:
            MatrixSelector(const MatrixDynSize& selectorMatrix)
                : Selector(selectorMatrix.rows())
                , m_selectorMatrix(selectorMatrix) {}

            ~MatrixSelector() override;

            virtual const VectorDynSize& select(const VectorDynSize& fullVector) final
            {
                iDynTree::toEigen(m_selected) = iDynTree::toEigen(m_selectorMatrix) * iDynTree::toEigen(fullVector);
                return m_selected;
            }
        };
        MatrixSelector::~MatrixSelector(){}

    //
    // Implementation of TimeVaryingGradient
    //

        class TimeVaryingGradient : public TimeVaryingVector {
            MatrixDynSize m_selectorMatrix, m_weightMatrix;
            MatrixDynSize m_hessianMatrix, m_gradientSubMatrix;
            std::shared_ptr<TimeVaryingVector> m_desiredTrajectory;
            VectorDynSize m_outputVector;
        public:
            TimeVaryingGradient(const MatrixDynSize& selectorMatrix)
            : m_selectorMatrix(selectorMatrix)
            , m_desiredTrajectory(nullptr)
            {
                m_weightMatrix.resize(m_selectorMatrix.rows(), m_selectorMatrix.rows());
                toEigen(m_weightMatrix).setIdentity();
                m_hessianMatrix.resize(m_selectorMatrix.cols(), m_selectorMatrix.cols());
                toEigen(m_hessianMatrix) = toEigen(m_selectorMatrix).transpose() * toEigen(m_selectorMatrix);
                m_gradientSubMatrix.resize(m_selectorMatrix.rows(), m_selectorMatrix.cols());
                toEigen(m_gradientSubMatrix) = -1 * toEigen(m_selectorMatrix);
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
                toEigen(m_hessianMatrix) = toEigen(m_selectorMatrix).transpose() * toEigen(m_weightMatrix) * toEigen(m_selectorMatrix);
                toEigen(m_gradientSubMatrix) = -0.5 * (toEigen(m_weightMatrix).transpose() * toEigen(m_selectorMatrix) + toEigen(m_weightMatrix) * toEigen(m_selectorMatrix));

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

                if (desiredPoint.size() != m_gradientSubMatrix.rows()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The specified desired point at time: " << time << " has size not matching the specified selector.";
                    reportError("TimeVaryingGradient", "getObject", errorMsg.str().c_str());
                    isValid = false;
                    m_outputVector.zero();
                    return m_outputVector;
                }

                toEigen(m_outputVector) = toEigen(desiredPoint).transpose() * toEigen(m_gradientSubMatrix);

                isValid = true;
                return m_outputVector;
            }

            bool updateSelector(const MatrixDynSize& selector) {
                if ((selector.rows() != m_selectorMatrix.rows()) || (selector.cols() != m_selectorMatrix.cols())) {
                    reportError("TimeVaryingGradient", "getObject", "The new selector dimensionsions do not match the old one.");
                    return false;
                }
                m_selectorMatrix = selector;
                toEigen(m_hessianMatrix) = toEigen(m_selectorMatrix).transpose() * toEigen(m_weightMatrix) * toEigen(m_selectorMatrix);
                toEigen(m_gradientSubMatrix) = -0.5 * (toEigen(m_weightMatrix).transpose() * toEigen(m_selectorMatrix) + toEigen(m_weightMatrix) * toEigen(m_selectorMatrix));

                return true;
            }

            const MatrixDynSize& selector() {
                return m_selectorMatrix;
            }

            const MatrixDynSize& hessianMatrix() {
                return m_hessianMatrix;
            }

            const MatrixDynSize& weightMatrix() {
                return m_weightMatrix;
            }

            std::shared_ptr<TimeVaryingVector> desiredTrajectory() {
                return m_desiredTrajectory;
            }
        };
        TimeVaryingGradient::~TimeVaryingGradient() {};

        //
        // End of implementation of TimeVaryingGradient
        //

        //
        // Definition of pimpl
        //
        class L2NormCost::L2NormCostImplementation {
            public:
            std::shared_ptr<TimeVaryingGradient> stateGradient = nullptr, controlGradient = nullptr;
            std::shared_ptr<TimeInvariantMatrix> stateHessian, controlHessian;
            std::shared_ptr<Selector> stateSelector_ptr, controlSelector_ptr;
            iDynTree::VectorDynSize stateCostBuffer, controlCostBuffer;

            void initializeHessianAndGradient(const MatrixDynSize &stateSelector, const MatrixDynSize &controlSelector) {
                if ((stateSelector.rows() != 0) && (stateSelector.cols() != 0)) {
                    stateGradient = std::make_shared<TimeVaryingGradient>(stateSelector);
                    stateHessian = std::make_shared<TimeInvariantMatrix>();
                    stateHessian->get().resize(stateSelector.cols(), stateSelector.cols());
                    stateHessian->get() = stateGradient->hessianMatrix();
                    stateCostBuffer.resize(stateSelector.rows());
                    stateCostBuffer.zero();
                } else {
                    stateGradient = nullptr;
                }

                if ((controlSelector.rows() != 0) && (controlSelector.cols() != 0)) {
                    controlGradient = std::make_shared<TimeVaryingGradient>(controlSelector);
                    controlHessian = std::make_shared<TimeInvariantMatrix>();
                    controlHessian->get().resize(controlSelector.cols(), controlSelector.cols());
                    controlHessian->get() = controlGradient->hessianMatrix();
                    controlCostBuffer.resize(controlSelector.rows());
                    controlCostBuffer.zero();
                } else {
                    controlGradient = nullptr;
                }
            }

            void initializeHessianAndGradient(unsigned int stateDimension, unsigned int controlDimension) {
                MatrixDynSize stateSelector(stateDimension, stateDimension), controlSelector(controlDimension, controlDimension);
                toEigen(stateSelector).setIdentity();
                toEigen(controlSelector).setIdentity();
                initializeHessianAndGradient(stateSelector, controlSelector);
            }

            void initializeHessianAndGradient(const IndexRange &stateSelector, unsigned int totalStateDimension, const IndexRange &controlSelector, unsigned int totalControlDimension) {
                MatrixDynSize stateSelectorMatrix, controlSelectorMatrix;

                if (stateSelector.isValid()) {
                    stateSelectorMatrix.resize(static_cast<unsigned int>(stateSelector.size), totalStateDimension);
                    iDynTree::toEigen(stateSelectorMatrix).block(0, stateSelector.offset, stateSelector.size, stateSelector.size).setIdentity();
                } else {
                    stateSelectorMatrix.resize(0,0);
                }

                if (controlSelector.isValid()) {
                    controlSelectorMatrix.resize(static_cast<unsigned int>(controlSelector.size), totalControlDimension);
                    iDynTree::toEigen(controlSelectorMatrix).block(0, controlSelector.offset, controlSelector.size, controlSelector.size).setIdentity();
                } else {
                    controlSelectorMatrix.resize(0,0);
                }

                initializeHessianAndGradient(stateSelectorMatrix, controlSelectorMatrix);
            }

        };

        //
        //End of definition of pimpl
        //

        //
        //Implementation of L2NormCost
        //

        L2NormCost::L2NormCost(const std::string &name, unsigned int stateDimension, unsigned int controlDimension)
        : QuadraticLikeCost(name)
        , m_pimpl(new L2NormCostImplementation)
        {
            assert(m_pimpl);

            m_pimpl->initializeHessianAndGradient(stateDimension, controlDimension);

            if (m_pimpl->stateGradient) {
                m_timeVaryingStateHessian = m_pimpl->stateHessian;
                m_timeVaryingStateGradient = m_pimpl->stateGradient;
                IndexRange stateRange;
                stateRange.offset = 0;
                stateRange.size = stateDimension;
                m_pimpl->stateSelector_ptr = std::make_shared<IndexSelector>(stateRange);
            }

            if (m_pimpl->controlGradient) {
                m_timeVaryingControlHessian = m_pimpl->controlHessian;
                m_timeVaryingControlGradient = m_pimpl->controlGradient;
                IndexRange controlRange;
                controlRange.offset = 0;
                controlRange.size = stateDimension;
                m_pimpl->controlSelector_ptr = std::make_shared<IndexSelector>(controlRange);
            }
        }

        L2NormCost::L2NormCost(const std::string &name, const MatrixDynSize &stateSelector, const MatrixDynSize &controlSelector)
        : QuadraticLikeCost(name)
        , m_pimpl(new L2NormCostImplementation)
        {
            m_pimpl->initializeHessianAndGradient(stateSelector, controlSelector);

            if (m_pimpl->stateGradient) {
                m_timeVaryingStateHessian = m_pimpl->stateHessian;
                m_timeVaryingStateGradient = m_pimpl->stateGradient;
                m_pimpl->stateSelector_ptr = std::make_shared<MatrixSelector>(stateSelector);
            }

            if (m_pimpl->controlGradient) {
                m_timeVaryingControlHessian = m_pimpl->controlHessian;
                m_timeVaryingControlGradient = m_pimpl->controlGradient;
                m_pimpl->controlSelector_ptr = std::make_shared<MatrixSelector>(controlSelector);
            }
        }

        L2NormCost::L2NormCost(const std::string &name, const IndexRange &stateSelector, unsigned int totalStateDimension, const IndexRange &controlSelector, unsigned int totalControlDimension)
            : QuadraticLikeCost (name)
            , m_pimpl(new L2NormCostImplementation)
        {
            m_pimpl->initializeHessianAndGradient(stateSelector, totalStateDimension, controlSelector, totalControlDimension);

            if (m_pimpl->stateGradient) {
                m_timeVaryingStateHessian = m_pimpl->stateHessian;
                m_timeVaryingStateGradient = m_pimpl->stateGradient;
                m_pimpl->stateSelector_ptr = std::make_shared<IndexSelector>(stateSelector);
            }

            if (m_pimpl->controlGradient) {
                m_timeVaryingControlHessian = m_pimpl->controlHessian;
                m_timeVaryingControlGradient = m_pimpl->controlGradient;
                m_pimpl->controlSelector_ptr = std::make_shared<IndexSelector>(controlSelector);
            }
        }

        L2NormCost::~L2NormCost()
        {
            if (m_pimpl){
                delete m_pimpl;
                m_pimpl = nullptr;
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

            if (desiredPoint.size() != (m_pimpl->stateGradient->selector().rows())) {
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

            if (desiredPoint.size() != (m_pimpl->controlGradient->selector().rows())) {
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
            m_pimpl->stateHessian->get() = m_pimpl->stateGradient->hessianMatrix();
            return true;
        }

        bool L2NormCost::updatControlSelector(const MatrixDynSize &controlSelector)
        {
            if (!(m_pimpl->controlGradient->updateSelector(controlSelector))) {
                reportError("L2NormCost", "updatControlSelector", "Error when updating state selector.");
                return false;
            }
            m_pimpl->controlHessian->get() = m_pimpl->controlGradient->hessianMatrix();
            return true;
        }

        bool L2NormCost::costEvaluation(double time, const VectorDynSize &state, const VectorDynSize &control, double &costValue)
        {
            double stateCost = 0, controlCost = 0;

            bool isValid = false;
            if (m_pimpl->stateGradient) {

                if (state.size() != m_pimpl->stateGradient->selector().cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state dimension is not the one expected from the specified selector.";
                    reportError("L2NormCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                iDynTree::toEigen(m_pimpl->stateCostBuffer) = iDynTree::toEigen(m_pimpl->stateSelector_ptr->select(state));

                if (m_pimpl->stateGradient->desiredTrajectory()) {
                    const VectorDynSize& desired = m_pimpl->stateGradient->desiredTrajectory()->get(time, isValid);

                    if (!isValid) {
                        std::ostringstream errorMsg;
                        errorMsg << "Unable to retrieve a valid desired state at time: " << time << ".";
                        reportError("L2NormCost", "costEvaluation", errorMsg.str().c_str());
                        return false;
                    }

                    if (m_pimpl->stateSelector_ptr->size() != desired.size()) {
                        std::ostringstream errorMsg;
                        errorMsg << "The desired state dimension does not match the size of the selector at time: " << time << ".";
                        reportError("L2NormCost", "costEvaluation", errorMsg.str().c_str());
                        return false;
                    }

                    iDynTree::toEigen(m_pimpl->stateCostBuffer) -= iDynTree::toEigen(desired);
                }

                stateCost = 0.5 * iDynTree::toEigen(m_pimpl->stateCostBuffer).transpose() * (iDynTree::toEigen(m_pimpl->stateGradient->weightMatrix()) * iDynTree::toEigen(m_pimpl->stateCostBuffer));
            }

            if (m_pimpl->controlGradient) {

                if (control.size() != m_pimpl->controlGradient->selector().cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control dimension is not the one expected from the specified selector.";
                    reportError("L2NormCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                iDynTree::toEigen(m_pimpl->controlCostBuffer) = iDynTree::toEigen(m_pimpl->controlSelector_ptr->select(control));

                if (m_pimpl->controlGradient->desiredTrajectory()) {
                    const VectorDynSize& desired = m_pimpl->controlGradient->desiredTrajectory()->get(time, isValid);

                    if (!isValid) {
                        std::ostringstream errorMsg;
                        errorMsg << "Unable to retrieve a valid desired control at time: " << time << ".";
                        reportError("L2NormCost", "costEvaluation", errorMsg.str().c_str());
                        return false;
                    }

                    if (m_pimpl->controlSelector_ptr->size() != desired.size()) {
                        std::ostringstream errorMsg;
                        errorMsg << "The desired control dimension does not match the size of the selector at time: " << time << ".";
                        reportError("L2NormCost", "costEvaluation", errorMsg.str().c_str());
                        return false;
                    }

                    iDynTree::toEigen(m_pimpl->controlCostBuffer) -= iDynTree::toEigen(desired);
                }

                controlCost = 0.5 * iDynTree::toEigen(m_pimpl->controlCostBuffer).transpose() * (iDynTree::toEigen(m_pimpl->controlGradient->weightMatrix()) * iDynTree::toEigen(m_pimpl->controlCostBuffer));
            }

            costValue = stateCost + controlCost;

            return true;
        }

    }
}

