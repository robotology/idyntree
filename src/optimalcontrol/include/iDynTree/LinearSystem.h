// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_LINEARSYSTEM_H
#define IDYNTREE_OPTIMALCONTROL_LINEARSYSTEM_H

#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/TimeVaryingObject.h>
#include <iDynTree/SparsityStructure.h>
#include <memory>

namespace iDynTree {

    class MatrixDynSize;

    namespace optimalcontrol {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class LinearSystem 
        : public iDynTree::optimalcontrol::DynamicalSystem {

        public:

            LinearSystem(size_t stateSize, size_t controlSize);

            LinearSystem(size_t stateSize, size_t controlSize,
                         const iDynTree::optimalcontrol::SparsityStructure& stateSparsity,
                         const iDynTree::optimalcontrol::SparsityStructure& controlSparsity);

            LinearSystem(const LinearSystem& other) = delete;

            ~LinearSystem() override;

            // time invariant system: single matrix
            bool setStateMatrix(const iDynTree::MatrixDynSize& stateMatrix);

            bool setControlMatrix(const iDynTree::MatrixDynSize& controlMatrix);

            //time variant case
            bool setStateMatrix(std::shared_ptr<TimeVaryingMatrix> stateMatrix);

            bool setControlMatrix(std::shared_ptr<TimeVaryingMatrix> controlMatrix);

            virtual bool dynamics(const VectorDynSize& state,
                                  double time,
                                  VectorDynSize& stateDynamics) final;

            virtual bool dynamicsStateFirstDerivative(const VectorDynSize& state,
                                                      double time,
                                                      MatrixDynSize& dynamicsDerivative) final;

            virtual bool dynamicsControlFirstDerivative(const VectorDynSize& state,
                                                        double time,
                                                        MatrixDynSize& dynamicsDerivative) final;

            virtual bool dynamicsStateFirstDerivativeSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity) final;

            virtual bool dynamicsControlFirstDerivativeSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity) final;

            virtual bool dynamicsSecondPartialDerivativeWRTState(double time,
                                                                 const iDynTree::VectorDynSize& state,
                                                                 const iDynTree::VectorDynSize& lambda,
                                                                 iDynTree::MatrixDynSize& partialDerivative) final;

            virtual bool dynamicsSecondPartialDerivativeWRTControl(double time,
                                                                   const iDynTree::VectorDynSize& state,
                                                                   const iDynTree::VectorDynSize& lambda,
                                                                   iDynTree::MatrixDynSize& partialDerivative) final;

            virtual bool dynamicsSecondPartialDerivativeWRTStateControl(double time,
                                                                        const iDynTree::VectorDynSize& state,
                                                                        const iDynTree::VectorDynSize& lambda,
                                                                        iDynTree::MatrixDynSize& partialDerivative) final;

            virtual bool dynamicsSecondPartialDerivativeWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity) final;

            virtual bool dynamicsSecondPartialDerivativeWRTStateControlSparsity(iDynTree::optimalcontrol::SparsityStructure& stateControlSparsity) final;

            virtual bool dynamicsSecondPartialDerivativeWRTControlSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity) final;


        private:
            class LinearSystemPimpl;
            LinearSystemPimpl* m_pimpl;

        };
    }
}


#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_LINEARSYSTEM_H */
