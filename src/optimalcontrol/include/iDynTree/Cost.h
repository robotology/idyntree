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

#ifndef IDYNTREE_OPTIMALCONTROL_COST_H
#define IDYNTREE_OPTIMALCONTROL_COST_H

#include <string>

namespace iDynTree {

    class VectorDynSize;
    class MatrixDynSize;

    namespace optimalcontrol {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        /**
         * @brief The Cost virtual class definition.
         * Publicly inherit from this class to specify a Cost to be used in a optimal control problem.
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
             * @brief costFirstPartialDerivativeWRTState
             * @param time
             * @param state
             * @param control
             * @param partialDerivative
             * @return
             */
            virtual bool costFirstPartialDerivativeWRTState(double time,
                                                            const iDynTree::VectorDynSize& state,
                                                            const iDynTree::VectorDynSize& control,
                                                            iDynTree::VectorDynSize& partialDerivative);

            virtual bool costFirstPartialDerivativeWRTControl(double time,
                                                              const iDynTree::VectorDynSize& state,
                                                              const iDynTree::VectorDynSize& control,
                                                              iDynTree::VectorDynSize& partialDerivative);

            virtual bool costSecondPartialDerivativeWRTState(double time,
                                                             const iDynTree::VectorDynSize& state,
                                                             const iDynTree::VectorDynSize& control,
                                                             iDynTree::MatrixDynSize& partialDerivative);

            virtual bool costSecondPartialDerivativeWRTControl(double time,
                                                               const iDynTree::VectorDynSize& state,
                                                               const iDynTree::VectorDynSize& control,
                                                               iDynTree::MatrixDynSize& partialDerivative);


            virtual bool costSecondPartialDerivativeWRTStateControl(double time,
                                                                    const iDynTree::VectorDynSize& state,
                                                                    const iDynTree::VectorDynSize& control,
                                                                    iDynTree::MatrixDynSize& partialDerivative);


        private:

            std::string m_costName;
        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_COST_H */
