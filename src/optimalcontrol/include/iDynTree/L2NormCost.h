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


#ifndef IDYNTREE_OPTIMALCONTROL_L2NORMCOST_H
#define IDYNTREE_OPTIMALCONTROL_L2NORMCOST_H

#include <iDynTree/Cost.h>
#include <iDynTree/TimeVaryingObject.h>
#include <iDynTree/Core/Utils.h>
#include <memory>
#include <string>

namespace iDynTree {

    class MatrixDynSize;
    class VectorDynSize;

    namespace optimalcontrol {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class L2NormCost : public Cost {
            class L2NormCostImplementation;
            L2NormCostImplementation *m_pimpl;

        public:

            L2NormCost(const std::string& name, unsigned int stateDimension, unsigned int controlDimension); //assume identity as selector. Set the dimension to 0 to avoid weighting either the state or the control

            L2NormCost(const std::string& name, const MatrixDynSize& stateSelector, const MatrixDynSize& controlSelector); //The selector premultiplies the state and the control in the L2-norm. Set some dimension to 0 to avoid weighting either the state or the control

            L2NormCost(const std::string& name, const IndexRange& stateSelector, unsigned int totalStateDimension,
                       const IndexRange& controlSelector, unsigned int totalControlDimension); //Pass an invalid range to avoid weighting either the state or the control.

            L2NormCost(const L2NormCost& other) = delete;

            ~L2NormCost();

            bool setStateWeight(const MatrixDynSize& stateWeights);

            bool setStateWeight(const VectorDynSize& stateWeights);

            bool setStateDesiredPoint(const VectorDynSize& desiredPoint);

            bool setStateDesiredTrajectory(std::shared_ptr<TimeVaryingVector> stateDesiredTrajectory);

            bool setControlWeight(const MatrixDynSize& controlWeights);

            bool setControlWeight(const VectorDynSize& controlWeights);

            bool setControlDesiredPoint(const VectorDynSize& desiredPoint);

            bool setControlDesiredTrajectory(std::shared_ptr<TimeVaryingVector> controlDesiredTrajectory);

            bool updatStateSelector(const MatrixDynSize& stateSelector);

            bool updatControlSelector(const MatrixDynSize& controlSelector);

            virtual bool costEvaluation(double time,
                                        const iDynTree::VectorDynSize& state,
                                        const iDynTree::VectorDynSize& control,
                                        double& costValue) final;

            virtual bool costFirstPartialDerivativeWRTState(double time,
                                                            const iDynTree::VectorDynSize& state, const iDynTree::VectorDynSize&,
                                                            iDynTree::VectorDynSize& partialDerivative) final;

            virtual bool costFirstPartialDerivativeWRTControl(double time, const iDynTree::VectorDynSize&,
                                                              const iDynTree::VectorDynSize& control,
                                                              iDynTree::VectorDynSize& partialDerivative) final;

            virtual bool costSecondPartialDerivativeWRTState(double,
                                                             const iDynTree::VectorDynSize& state, const iDynTree::VectorDynSize&,
                                                             iDynTree::MatrixDynSize& partialDerivative) final;

            virtual bool costSecondPartialDerivativeWRTControl(double, const iDynTree::VectorDynSize&,
                                                               const iDynTree::VectorDynSize& control,
                                                               iDynTree::MatrixDynSize& partialDerivative) final;


            virtual bool costSecondPartialDerivativeWRTStateControl(double,
                                                                    const iDynTree::VectorDynSize& state,
                                                                    const iDynTree::VectorDynSize& control,
                                                                    iDynTree::MatrixDynSize& partialDerivative) final;

            virtual bool costSecondPartialDerivativeWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity) final;

            virtual bool costSecondPartialDerivativeWRTStateControlSparsity(iDynTree::optimalcontrol::SparsityStructure& stateControlSparsity) final;

            virtual bool costSecondPartialDerivativeWRTControlSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity) final;
        };

    }
}

#endif // IDYNTREE_OPTIMALCONTROL_L2NORMCOST_H
