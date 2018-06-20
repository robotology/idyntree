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

#ifndef IDYNTREE_OPTIMALCONTROL_QUADRATICCOST_H
#define IDYNTREE_OPTIMALCONTROL_QUADRATICCOST_H

#include <iDynTree/Cost.h>

#include <iDynTree/Core/MatrixDynSize.h>

namespace iDynTree {
    namespace optimalcontrol {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        /*@
         * Models a cost of the form
         * \f[
         * x^\top Q x + u^\top R u
         * \f]
         * where \f$x \in \mathbb{R}^n\f$ is the state and
         * \f$u \in \mathbb{R}^n\f$ is the control
         */
        class QuadraticCost
        : public Cost {

            QuadraticCost(const iDynTree::MatrixDynSize& Q,
                          const iDynTree::MatrixDynSize& R,
                          const std::string& costName);


            virtual bool costEvaluation(double time,
                                        const iDynTree::VectorDynSize& state,
                                        const iDynTree::VectorDynSize& control,
                                        double& costValue) override;

            virtual bool costFirstPartialDerivativeWRTState(double time,
                                                            const iDynTree::VectorDynSize& state,
                                                            const iDynTree::VectorDynSize& control,
                                                            iDynTree::VectorDynSize& partialDerivative) override;

            virtual bool costFirstPartialDerivativeWRTControl(double time,
                                                              const iDynTree::VectorDynSize& state,
                                                              const iDynTree::VectorDynSize& control,
                                                              iDynTree::VectorDynSize& partialDerivative) override;

            virtual bool costSecondPartialDerivativeWRTState(double time,
                                                             const iDynTree::VectorDynSize& state,
                                                             const iDynTree::VectorDynSize& control,
                                                             iDynTree::MatrixDynSize& partialDerivative) override;

            virtual bool costSecondPartialDerivativeWRTControl(double time,
                                                               const iDynTree::VectorDynSize& state,
                                                               const iDynTree::VectorDynSize& control,
                                                               iDynTree::MatrixDynSize& partialDerivative) override;


            virtual bool costSecondPartialDerivativeWRTStateControl(double time,
                                                                    const iDynTree::VectorDynSize& state,
                                                                    const iDynTree::VectorDynSize& control,
                                                                    iDynTree::MatrixDynSize& partialDerivative) override;

        protected:
            iDynTree::MatrixDynSize m_stateCostMatrix;
            iDynTree::MatrixDynSize m_controlCostMatrix;
            iDynTree::MatrixDynSize m_stateControlCostDerivativeMatrix;

        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_QUADRATICCOST_H */
