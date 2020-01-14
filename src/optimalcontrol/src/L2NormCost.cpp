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
            MatrixDynSize m_selectionMatrix;
            SparsityStructure m_correspondingHessianSparsity;
        public:
            Selector(unsigned int selectedSize) : m_selected(selectedSize) { }

            virtual ~Selector();

            virtual const VectorDynSize& select(const VectorDynSize& fullVector) = 0;

            virtual const MatrixDynSize& asSelectorMatrix() {
                return m_selectionMatrix;
            }

            const SparsityStructure& sparsity() {
                return m_correspondingHessianSparsity;
            }

            void setSparsity(const SparsityStructure& sparsity) {
                m_correspondingHessianSparsity = sparsity;
            }

            unsigned int size() const {
                return m_selected.size();
            }

        };
        Selector::~Selector(){}

        class IndexSelector : public Selector {
            IndexRange m_selectedIndex;
        public:
            IndexSelector(const IndexRange& selectedRange, unsigned int totalSize)
                : Selector(static_cast<unsigned int>(selectedRange.size))
                , m_selectedIndex(selectedRange)
            {
                m_selectionMatrix.resize(static_cast<unsigned int>(selectedRange.size), totalSize);
                toEigen(m_selectionMatrix).block(0, selectedRange.offset, selectedRange.size, selectedRange.size).setIdentity();
                m_correspondingHessianSparsity.addDenseBlock(selectedRange, selectedRange);
            }

            ~IndexSelector() override;

            virtual const VectorDynSize& select(const VectorDynSize& fullVector) final
            {
                m_selected = make_span(fullVector).subspan(m_selectedIndex.offset, m_selectedIndex.size);
                return m_selected;
            }

        };
        IndexSelector::~IndexSelector(){}

        class MatrixSelector : public Selector {
        public:
            MatrixSelector(const MatrixDynSize& selectorMatrix)
                : Selector(selectorMatrix.rows())
            {
                m_selectionMatrix = selectorMatrix;
                m_correspondingHessianSparsity.addDenseBlock(0, 0, selectorMatrix.cols(), selectorMatrix.cols()); //dense
            }

            ~MatrixSelector() override;

            virtual const VectorDynSize& select(const VectorDynSize& fullVector) final
            {
                iDynTree::toEigen(m_selected) = iDynTree::toEigen(m_selectionMatrix) * iDynTree::toEigen(fullVector);
                return m_selected;
            }
        };
        MatrixSelector::~MatrixSelector(){}

        //
        // End of implementation of selectors
        //

        class CostAttributes {
            bool m_valid = false;
            SparsityStructure m_emptySparsity; //dummy sparsity in case this cost is not initialized

        public:
            std::shared_ptr<TimeVaryingVector> desiredTrajectory;
            iDynTree::MatrixDynSize gradientSubMatrix, hessianMatrix;
            iDynTree::MatrixDynSize weightMatrix;
            std::shared_ptr<Selector> selector;
            iDynTree::VectorDynSize buffer;

            void initializeBuffers(unsigned int rangeSize, unsigned int totalDimension) {
                this->buffer.resize(rangeSize);
                this->buffer.zero();
                this->desiredTrajectory = std::make_shared<TimeInvariantVector>(this->buffer);
                this->weightMatrix.resize(rangeSize, rangeSize);
                toEigen(this->weightMatrix).setIdentity();
                this->gradientSubMatrix.resize(totalDimension, rangeSize);
                toEigen(this->gradientSubMatrix) = toEigen(this->selector->asSelectorMatrix()).transpose();
                this->hessianMatrix.resize(totalDimension, totalDimension);
                toEigen(this->hessianMatrix) = toEigen(this->selector->asSelectorMatrix()).transpose() * toEigen(this->selector->asSelectorMatrix());
                this->m_valid = true;
            }

            void initializeCost(const IndexRange& selectedRange, unsigned int totalDimension) {
                this->selector = std::make_shared<IndexSelector>(selectedRange, totalDimension);
                unsigned int rangeSize = static_cast<unsigned int>(selectedRange.size);
                initializeBuffers(rangeSize, totalDimension);
            }

            void initializeCost(const MatrixDynSize& inputSelector) {
                this->selector = std::make_shared<MatrixSelector>(inputSelector);
                unsigned int rangeSize = inputSelector.rows();
                unsigned int totalDimension = inputSelector.cols();
                initializeBuffers(rangeSize, totalDimension);
            }


            bool setDesiredTrajectory(std::shared_ptr<TimeVaryingVector> inputTrajectory) {

                this->desiredTrajectory = inputTrajectory;
                return true;
            }

            bool setWeightMatrix(const MatrixDynSize &weights)
            {
                if (weights.rows() != weights.cols()) {
                    reportError("L2NormCost", "setWeightMatrix", "The weights matrix is supposed to be squared.");
                    return false;
                }

                if (weights.cols() != this->selector->size()) {
                    reportError("L2NormCost", "setWeightMatrix", "The weights matrix dimensions do not match those of the specified selector.");
                    return false;
                }

                this->weightMatrix = weights;
                toEigen(this->gradientSubMatrix) = 0.5 * toEigen(this->selector->asSelectorMatrix()).transpose() * (toEigen(this->weightMatrix) + toEigen(this->weightMatrix).transpose());
                toEigen(this->hessianMatrix) = toEigen(this->gradientSubMatrix) * toEigen(this->selector->asSelectorMatrix());

                return true;
            }

            bool setWeightMatrix(const VectorDynSize &weights)
            {
                if (weights.size() != this->selector->size()) {
                    reportError("L2NormCost", "setWeightMatrix", "The weights matrix dimensions do not match those of the specified selector.");
                    return false;
                }

                iDynTree::toEigen(this->weightMatrix) = iDynTree::toEigen(weights).asDiagonal();
                toEigen(this->gradientSubMatrix) = toEigen(this->selector->asSelectorMatrix()).transpose() * toEigen(this->weightMatrix);
                toEigen(this->hessianMatrix) = toEigen(this->gradientSubMatrix) * toEigen(this->selector->asSelectorMatrix());

                return true;
            }

            bool changeSelector(const MatrixDynSize& inputSelector) {
                if ((inputSelector.rows() != this->selector->size()) || (inputSelector.cols() != this->selector->asSelectorMatrix().cols())) {
                    reportError("L2NormCost", "getObject", "The new selector dimensionsions do not match the old one.");
                    return false;
                }
                this->selector.reset(new MatrixSelector(inputSelector));
                toEigen(this->gradientSubMatrix) = 0.5 * toEigen(this->selector->asSelectorMatrix()).transpose() * (toEigen(this->weightMatrix) + toEigen(this->weightMatrix).transpose());
                toEigen(this->hessianMatrix) = toEigen(this->gradientSubMatrix) * toEigen(this->selector->asSelectorMatrix());

                return true;
            }

            bool isValid() {
                return m_valid;
            }

            bool evaluateCost(double time, const VectorDynSize& values, double& costValue, const std::string &label) {
                if (!isValid()) {
                    costValue = 0.0;
                    return true;
                }

                if (values.size() != selector->asSelectorMatrix().cols()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The " << label <<" dimension is not the one expected from the specified selector.";
                    reportError("L2NormCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                bool isValid;
                const VectorDynSize& desired = desiredTrajectory->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to retrieve a valid desired " << label <<" at time: " << time << ".";
                    reportError("L2NormCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                if (selector->size() != desired.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The desired " << label <<" dimension does not match the size of the selector at time: " << time << ".";
                    reportError("L2NormCost", "costEvaluation", errorMsg.str().c_str());
                    return false;
                }

                iDynTree::toEigen(buffer) = iDynTree::toEigen(selector->select(values)) - iDynTree::toEigen(desired);


                costValue = 0.5 * iDynTree::toEigen(buffer).transpose() * (iDynTree::toEigen(weightMatrix) * iDynTree::toEigen(buffer));

                return true;
            }

            bool evaluateGradient(double time, const VectorDynSize& values, VectorDynSize& partialDerivative, const std::string &label) {
                partialDerivative.resize(values.size());

                if (!isValid()) {
                    partialDerivative.zero();
                    return true;
                }

                if (values.size() != selector->asSelectorMatrix().cols()) {
                    std::ostringstream errorMsg, errorTag;
                    errorTag << "costFirstPartialDerivativeWRT" << label;
                    errorMsg << "The " << label <<" dimension is not the one expected from the specified selector.";
                    reportError("L2NormCost", errorTag.str().c_str(), errorMsg.str().c_str());
                    return false;
                }

                bool isValid;
                const VectorDynSize& desired = desiredTrajectory->get(time, isValid);

                if (!isValid) {
                    std::ostringstream errorMsg, errorTag;
                    errorTag << "costFirstPartialDerivativeWRT" << label;
                    errorMsg << "Unable to retrieve a valid desired " << label <<" at time: " << time << ".";
                    reportError("L2NormCost", errorTag.str().c_str(), errorMsg.str().c_str());
                    return false;
                }

                if (selector->size() != desired.size()) {
                    std::ostringstream errorMsg, errorTag;
                    errorTag << "costFirstPartialDerivativeWRT" << label;;
                    errorMsg << "The desired " << label <<" dimension does not match the size of the selector at time: " << time << ".";
                    reportError("L2NormCost", errorTag.str().c_str(), errorMsg.str().c_str());
                    return false;
                }

                iDynTree::toEigen(buffer) = iDynTree::toEigen(selector->select(values)) - iDynTree::toEigen(desired);
                iDynTree::toEigen(partialDerivative) = iDynTree::toEigen(gradientSubMatrix) * iDynTree::toEigen(buffer);

                return true;
            }

            bool getHessian(const VectorDynSize& values, MatrixDynSize& partialDerivative, const std::string &label) {

                partialDerivative.resize(values.size(), values.size());

                if (!isValid()) {
                    partialDerivative.zero();
                    return true;
                }

                if (values.size() != selector->asSelectorMatrix().cols()) {
                    std::ostringstream errorMsg, errorTag;
                    errorTag << "costSecondPartialDerivativeWRT" << label;
                    errorMsg << "The " << label <<" dimension is not the one expected from the specified selector.";
                    reportError("L2NormCost", errorTag.str().c_str(), errorMsg.str().c_str());
                    return false;
                }

                partialDerivative = hessianMatrix;

                return true;

            }

            const SparsityStructure& getHessianSparsity() {
                if (!isValid()) {
                    return m_emptySparsity;
                }
                return selector->sparsity();
            }

        };

        //
        // Definition of pimpl
        //
        class L2NormCost::L2NormCostImplementation {
            public:
            CostAttributes stateCost, controlCost;
        };

        //
        //End of definition of pimpl
        //

        //
        //Implementation of L2NormCost
        //

        L2NormCost::L2NormCost(const std::string &name, unsigned int stateDimension, unsigned int controlDimension)
            : Cost(name)
            , m_pimpl(new L2NormCostImplementation)
        {
            if (stateDimension > 0) {
                IndexRange stateRange;
                stateRange.offset = 0;
                stateRange.size = stateDimension;

                m_pimpl->stateCost.initializeCost(stateRange, stateDimension);
            }

            if (controlDimension > 0) {
                IndexRange controlRange;
                controlRange.offset = 0;
                controlRange.size = controlDimension;

                m_pimpl->controlCost.initializeCost(controlRange, controlDimension);
            }
        }

        L2NormCost::L2NormCost(const std::string &name, const MatrixDynSize &stateSelector, const MatrixDynSize &controlSelector)
            : Cost(name)
            , m_pimpl(new L2NormCostImplementation)
        {
            if ((stateSelector.rows() != 0) && (stateSelector.cols() != 0)) {
                m_pimpl->stateCost.initializeCost(stateSelector);
            }

            if ((controlSelector.rows() != 0) && (controlSelector.cols() != 0)) {
                m_pimpl->controlCost.initializeCost(controlSelector);
            }
        }

        L2NormCost::L2NormCost(const std::string &name, const IndexRange &stateSelector, unsigned int totalStateDimension, const IndexRange &controlSelector, unsigned int totalControlDimension)
            : Cost (name)
            , m_pimpl(new L2NormCostImplementation)
        {
            if (stateSelector.isValid() && totalStateDimension > 0) {
                m_pimpl->stateCost.initializeCost(stateSelector, totalStateDimension);

            }

            if (controlSelector.isValid() && totalControlDimension > 0) {
                m_pimpl->controlCost.initializeCost(controlSelector, totalControlDimension);

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

            if (!(m_pimpl->stateCost.isValid())) {
                reportError("L2NormCost", "setStateWeight", "The state cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (!(m_pimpl->stateCost.setWeightMatrix(stateWeights))) {
                reportError("L2NormCost", "setStateWeight", "Error when specifying the state weights.");
                return false;
            }

            return true;
        }

        bool L2NormCost::setStateWeight(const VectorDynSize &stateWeights)
        {
            if (!(m_pimpl->stateCost.isValid())) {
                reportError("L2NormCost", "setStateWeight", "The state cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (!(m_pimpl->stateCost.setWeightMatrix(stateWeights))) {
                reportError("L2NormCost", "setStateWeight", "Error when specifying the state weights.");
                return false;
            }

            return true;
        }

        bool L2NormCost::setStateDesiredPoint(const VectorDynSize &desiredPoint)
        {
            if (!(m_pimpl->stateCost.isValid())) {
                reportError("L2NormCost", "setStateDesiredPoint", "The state cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (desiredPoint.size() != (m_pimpl->stateCost.selector->size())) {
                reportError("L2NormCost", "setStateDesiredPoint", "The desiredPoint size do not match the dimension of the specified selector.");
                return false;
            }

            std::shared_ptr<TimeInvariantVector> newTrajectory = std::make_shared<TimeInvariantVector>(desiredPoint);
            return m_pimpl->stateCost.setDesiredTrajectory(newTrajectory);
        }

        bool L2NormCost::setStateDesiredTrajectory(std::shared_ptr<TimeVaryingVector> stateDesiredTrajectory)
        {
            if (!(m_pimpl->stateCost.isValid())) {
                reportError("L2NormCost", "setStateDesiredTrajectory", "The state cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (!stateDesiredTrajectory) {
                reportError("L2NormCost", "setStateDesiredTrajectory", "Empty desired trajectory pointer.");
                return false;
            }

            return m_pimpl->stateCost.setDesiredTrajectory(stateDesiredTrajectory);
        }

        bool L2NormCost::setControlWeight(const MatrixDynSize &controlWeights)
        {
            if (controlWeights.rows() != controlWeights.cols()) {
                reportError("L2NormCost", "setControlWeight", "The controlWeights matrix is supposed to be squared.");
                return false;
            }

            if (!(m_pimpl->controlCost.isValid())) {
                reportError("L2NormCost", "setControlWeight", "The control cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (!(m_pimpl->controlCost.setWeightMatrix(controlWeights))) {
                reportError("L2NormCost", "setControlWeight", "Error when specifying the control weights.");
                return false;
            }

            return true;
        }

        bool L2NormCost::setControlWeight(const VectorDynSize &controlWeights)
        {
            if (!(m_pimpl->controlCost.isValid())) {
                reportError("L2NormCost", "setControlWeight", "The control cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (!(m_pimpl->controlCost.setWeightMatrix(controlWeights))) {
                reportError("L2NormCost", "setControlWeight", "Error when specifying the control weights.");
                return false;
            }

            return true;
        }

        bool L2NormCost::setControlDesiredPoint(const VectorDynSize &desiredPoint)
        {
            if (!(m_pimpl->controlCost.isValid())) {
                reportError("L2NormCost", "setControlDesiredPoint", "The control cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (desiredPoint.size() != (m_pimpl->controlCost.selector->size())) {
                reportError("L2NormCost", "setControlDesiredPoint", "The desiredPoint size do not match the dimension of the specified selector.");
                return false;
            }

            std::shared_ptr<TimeInvariantVector> newTrajectory = std::make_shared<TimeInvariantVector>(desiredPoint);
            return m_pimpl->controlCost.setDesiredTrajectory(newTrajectory);
        }

        bool L2NormCost::setControlDesiredTrajectory(std::shared_ptr<TimeVaryingVector> controlDesiredTrajectory)
        {
            if (!(m_pimpl->controlCost.isValid())) {
                reportError("L2NormCost", "setControlDesiredTrajectory", "The control cost portion has been deactivated, given the provided selectors.");
                return false;
            }

            if (!controlDesiredTrajectory) {
                reportError("L2NormCost", "setControlDesiredTrajectory", "Empty desired trajectory pointer.");
                return false;
            }

            return m_pimpl->controlCost.setDesiredTrajectory(controlDesiredTrajectory);
        }

        bool L2NormCost::updatStateSelector(const MatrixDynSize &stateSelector)
        {
            if (!(m_pimpl->stateCost.changeSelector(stateSelector))) {
                reportError("L2NormCost", "updatStateSelector", "Error when updating state selector.");
                return false;
            }
            return true;
        }

        bool L2NormCost::updatControlSelector(const MatrixDynSize &controlSelector)
        {
            if (!(m_pimpl->controlCost.changeSelector(controlSelector))) {
                reportError("L2NormCost", "updatControlSelector", "Error when updating state selector.");
                return false;
            }
            return true;
        }

        bool L2NormCost::costEvaluation(double time, const VectorDynSize &state, const VectorDynSize &control, double &costValue)
        {
            double stateCost = 0, controlCost = 0;

            if (!m_pimpl->stateCost.evaluateCost(time, state, stateCost, "state"))
                return false;

            if (!m_pimpl->controlCost.evaluateCost(time, control, controlCost, "control"))
                return false;

            costValue = stateCost + controlCost;

            return true;
        }

        bool L2NormCost::costFirstPartialDerivativeWRTState(double time, const VectorDynSize &state, const VectorDynSize &/*control*/, VectorDynSize &partialDerivative)
        {
            return m_pimpl->stateCost.evaluateGradient(time, state, partialDerivative, "state");
        }

        bool L2NormCost::costFirstPartialDerivativeWRTControl(double time, const VectorDynSize &/*state*/, const VectorDynSize &control, VectorDynSize &partialDerivative)
        {
            return m_pimpl->controlCost.evaluateGradient(time, control, partialDerivative, "control");
        }

        bool L2NormCost::costSecondPartialDerivativeWRTState(double /*time*/, const VectorDynSize &state, const VectorDynSize &/*control*/, MatrixDynSize &partialDerivative)
        {
            return m_pimpl->stateCost.getHessian(state, partialDerivative, "state");
        }

        bool L2NormCost::costSecondPartialDerivativeWRTControl(double /*time*/, const VectorDynSize &/*state*/, const VectorDynSize &control, MatrixDynSize &partialDerivative)
        {
            return m_pimpl->controlCost.getHessian(control, partialDerivative, "control");
        }

        bool L2NormCost::costSecondPartialDerivativeWRTStateControl(double /*time*/, const VectorDynSize &state, const VectorDynSize &control, MatrixDynSize &partialDerivative)
        {
            partialDerivative.resize(state.size(), control.size());
            partialDerivative.zero();
            return true;
        }

        bool L2NormCost::costSecondPartialDerivativeWRTStateSparsity(SparsityStructure &stateSparsity)
        {
            stateSparsity = m_pimpl->stateCost.getHessianSparsity();
            return true;
        }

        bool L2NormCost::costSecondPartialDerivativeWRTStateControlSparsity(SparsityStructure &stateControlSparsity)
        {
            stateControlSparsity.clear();
            return true;
        }

        bool L2NormCost::costSecondPartialDerivativeWRTControlSparsity(SparsityStructure &controlSparsity)
        {
            controlSparsity = m_pimpl->controlCost.getHessianSparsity();
            return true;
        }

    }
}

