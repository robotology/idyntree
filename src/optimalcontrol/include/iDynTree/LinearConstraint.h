/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_LINEARCONSTRAINT_H
#define IDYNTREE_OPTIMALCONTROL_LINEARCONSTRAINT_H

#include <iDynTree/Constraint.h>

#include <iDynTree/Core/MatrixDynSize.h>

namespace iDynTree {
    namespace optimalcontrol {

        /*@
         * Models a linear constraints on the state and/or control, i.e.
         * \f[
         *      lb \leq A \begin{bmatrix} x\\u\end{bmatrix} \leq ub
         * \f]
         */

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class LinearConstraint
        : public Constraint {
        public:

            virtual ~LinearConstraint();

            virtual bool evaluateConstraint(double time,
                                            const VectorDynSize& state,
                                            const VectorDynSize& control,
                                            VectorDynSize& constraint) override;

            virtual bool constraintJacobianWRTState(double time,
                                                    const VectorDynSize& state,
                                                    const VectorDynSize& control,
                                                    MatrixDynSize& jacobian) override;

            virtual bool constraintJacobianWRTControl(double time,
                                                      const VectorDynSize& state,
                                                      const VectorDynSize& control,
                                                      MatrixDynSize& jacobian) override;

        private:
            iDynTree::MatrixDynSize m_stateConstraintMatrix;
            iDynTree::MatrixDynSize m_controlConstraintMatrix;
        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_LINEARCONSTRAINT_H */
