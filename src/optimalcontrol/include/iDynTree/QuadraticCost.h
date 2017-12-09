/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_QUADRATICCOST_H
#define IDYNTREE_OPTIMALCONTROL_QUADRATICCOST_H

#include "Cost.h"

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
