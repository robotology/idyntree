// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_COST_H
#define IDYNTREE_OPTIMALCONTROL_COST_H

#include <string>
#include <iDynTree/SparsityStructure.h>


namespace iDynTree {

    class VectorDynSize;
    class MatrixDynSize;

    namespace optimalcontrol {

        /**
         * @brief The Cost virtual class definition.
         * Publicly inherit from this class to specify a Cost to be used in a optimal control problem.
         */

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class Cost {
        public:

            /**
             * @brief Cost constructor
             * @param[in] costName Univocal name of the cost.
             */
            Cost(const std::string& costName);

            virtual ~Cost();

            /**
             * @brief Get the name of the cost
             * @return The name of the cost
             */
            const std::string& name() const; //the name must not be changed

            /**
             * @brief Evaluate the cost function.
             * @param[in] time The time at which the cost is evaluated.
             * @param[in] state The state value with which the cost has to be evaluated.
             * @param[in] control The control value with which the cost has to be evaluated.
             * @param[out] costValue The cost value given the above inputs.
             * @return True if successfull, false otherwise.
             */
            virtual bool costEvaluation(double time,
                                        const iDynTree::VectorDynSize& state,
                                        const iDynTree::VectorDynSize& control,
                                        double& costValue) = 0;

            /**
             * @brief Evaluate cost first partial derivative wrt the state
             *
             * It is the result of \f$\frac{\partial g(t, x, u)}{\partial x}\f$
             * @param[in] time The time at which the partial derivative is computed.
             * @param[in] state The state value at which the partial derivative is computed.
             * @param[in] control The control value at which the partial derivative is computed.
             * @param[out] partialDerivative The output partial derivative.
             * @return True if successfull, false otherwise (or if not implemented).
             */
            virtual bool costFirstPartialDerivativeWRTState(double time,
                                                            const iDynTree::VectorDynSize& state,
                                                            const iDynTree::VectorDynSize& control,
                                                            iDynTree::VectorDynSize& partialDerivative);

            /**
             * @brief Evaluate cost first partial derivative wrt the control variables
             *
             * It is the result of \f$\frac{\partial g(t, x, u)}{\partial u}\f$
             * @param[in] time The time at which the partial derivative is computed.
             * @param[in] state The state value at which the partial derivative is computed.
             * @param[in] control The control value at which the partial derivative is computed.
             * @param[out] partialDerivative The output partial derivative.
             * @return True if successfull, false otherwise (or if not implemented).
             */
            virtual bool costFirstPartialDerivativeWRTControl(double time,
                                                              const iDynTree::VectorDynSize& state,
                                                              const iDynTree::VectorDynSize& control,
                                                              iDynTree::VectorDynSize& partialDerivative);

            /**
             * @brief Evaluate cost second partial derivative wrt the state variables
             *
             * It is the result of \f$\frac{\partial^2 g(t, x, u)}{\partial x^2}\f$
             * @param[in] time The time at which the partial derivative is computed.
             * @param[in] state The state value at which the partial derivative is computed.
             * @param[in] control The control value at which the partial derivative is computed.
             * @param[out] partialDerivative The output partial derivative.
             * @return True if successfull, false otherwise (or if not implemented).
             */
            virtual bool costSecondPartialDerivativeWRTState(double time,
                                                             const iDynTree::VectorDynSize& state,
                                                             const iDynTree::VectorDynSize& control,
                                                             iDynTree::MatrixDynSize& partialDerivative);


            /**
             * @brief Evaluate cost second partial derivative wrt the control
             *
             * It is the result of \f$\frac{\partial^2 g(t, x, u)}{\partial u^2}\f$
             * @param[in] time The time at which the partial derivative is computed.
             * @param[in] state The state value at which the partial derivative is computed.
             * @param[in] control The control value at which the partial derivative is computed.
             * @param[out] partialDerivative The output partial derivative.
             * @return True if successfull, false otherwise (or if not implemented).
             */
            virtual bool costSecondPartialDerivativeWRTControl(double time,
                                                               const iDynTree::VectorDynSize& state,
                                                               const iDynTree::VectorDynSize& control,
                                                               iDynTree::MatrixDynSize& partialDerivative);


            /**
             * @brief Evaluate cost second partial derivative wrt the state and control
             *
             * It is the result of \f$\frac{\partial^2 g(t, x, u)}{\partial x \partial u}\f$,
             * thus it has number of rows equals to the number of states and number of cols equal to the number of control inputs.
             * @param[in] time The time at which the partial derivative is computed.
             * @param[in] state The state value at which the partial derivative is computed.
             * @param[in] control The control value at which the partial derivative is computed.
             * @param[out] partialDerivative The output partial derivative.
             * @return True if successfull, false otherwise (or if not implemented).
             */
            virtual bool costSecondPartialDerivativeWRTStateControl(double time,
                                                                    const iDynTree::VectorDynSize& state,
                                                                    const iDynTree::VectorDynSize& control,
                                                                    iDynTree::MatrixDynSize& partialDerivative);

            /**
             * @brief Returns the set of nonzeros elements in terms of row and colun index, in the state hessian
             *
             * @warning No check is performed in the indeces. They need to be in the range [0, stateDimension) and [0, stateDimension) respectively.
             * @param[out] stateSparsity Sparsity structure of the partial derivative of the gradient wrt state variables.
             * @return true if the sparsity is available. False otherwise.
             */
            virtual bool costSecondPartialDerivativeWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity);

            /**
             * @brief Returns the set of nonzeros elements in terms of row and colun index, in the mixed hessian
             *
             * @warning No check is performed in the indeces. They need to be in the range [0, stateDimension) and [0, controlDimension) respectively.
             * @param[out] stateControlSparsity Sparsity structure of the partial derivative of the gradient wrt state and control variables.
             * @return true if the sparsity is available. False otherwise.
             */
            virtual bool costSecondPartialDerivativeWRTStateControlSparsity(iDynTree::optimalcontrol::SparsityStructure& stateControlSparsity);

            /**
             * @brief Returns the set of nonzeros elements in terms of row and colun index, in the control hessian
             *
             * @warning No check is performed in the indeces. They need to be in the range [0, constraintDimension) and [0, controlDimension) respectively.
             * @param[out] controlSparsity Sparsity structure of the partial derivative of the gradient wrt control variables.
             * @return true if the sparsity is available. False otherwise.
             */
            virtual bool costSecondPartialDerivativeWRTControlSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity);


        private:

            std::string m_costName;
        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_COST_H */
