// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_LINEARCONSTRAINT_H
#define IDYNTREE_OPTIMALCONTROL_LINEARCONSTRAINT_H

#include <iDynTree/Constraint.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/TimeVaryingObject.h>
#include <iDynTree/SparsityStructure.h>
#include <string>
#include <memory>

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

            LinearConstraint(size_t size, const std::string name);

            LinearConstraint(size_t size, const std::string name,
                             const SparsityStructure& stateSparsity,
                             const SparsityStructure& controlSparsity);

            virtual ~LinearConstraint() override;

            bool setStateConstraintMatrix(const MatrixDynSize& constraintMatrix);

            bool setControlConstraintMatrix(const MatrixDynSize& constraintMatrix);

            bool setStateConstraintMatrix(std::shared_ptr<TimeVaryingMatrix> constraintMatrix);

            bool setControlConstraintMatrix(std::shared_ptr<TimeVaryingMatrix> constraintMatrix);

            virtual bool evaluateConstraint(double time,
                                            const VectorDynSize& state,
                                            const VectorDynSize& control,
                                            VectorDynSize& constraint) final;// lu <= [Ax, Au][x;u] <= lU

            virtual bool constraintJacobianWRTState(double time,
                                                    const VectorDynSize& state,
                                                    const VectorDynSize& control,
                                                    MatrixDynSize& jacobian) final;

            virtual bool constraintJacobianWRTControl(double time,
                                                      const VectorDynSize& state,
                                                      const VectorDynSize& control,
                                                      MatrixDynSize& jacobian) final;

            virtual bool constraintJacobianWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity) final;

            virtual bool constraintJacobianWRTControlSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity) final;

            virtual bool constraintSecondPartialDerivativeWRTState(double time,
                                                                   const VectorDynSize& state,
                                                                   const VectorDynSize& control,
                                                                   const VectorDynSize& lambda,
                                                                   MatrixDynSize& hessian) final;

            virtual bool constraintSecondPartialDerivativeWRTControl(double time,
                                                                     const VectorDynSize& state,
                                                                     const VectorDynSize& control,
                                                                     const VectorDynSize& lambda,
                                                                     MatrixDynSize& hessian) final;


            virtual bool constraintSecondPartialDerivativeWRTStateControl(double time,
                                                                          const VectorDynSize& state,
                                                                          const VectorDynSize& control,
                                                                          const VectorDynSize& lambda,
                                                                          MatrixDynSize& hessian) final;

            virtual bool constraintSecondPartialDerivativeWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity) final;

            virtual bool constraintSecondPartialDerivativeWRTStateControlSparsity(iDynTree::optimalcontrol::SparsityStructure& stateControlSparsity) final;

            virtual bool constraintSecondPartialDerivativeWRTControlSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity) final;

        private:
            class LinearConstraintImplementation;
            LinearConstraintImplementation* m_pimpl;
        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_LINEARCONSTRAINT_H */
