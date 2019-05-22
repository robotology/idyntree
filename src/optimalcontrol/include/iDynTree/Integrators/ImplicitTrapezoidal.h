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

#ifndef IDYNTREE_OPTIMALCONTROL_IMPLICITTRAPEZOIDAL_H
#define IDYNTREE_OPTIMALCONTROL_IMPLICITTRAPEZOIDAL_H

#include <iDynTree/Integrators/FixedStepIntegrator.h>

namespace iDynTree {
    namespace optimalcontrol {

        class DynamicalSystem;

        namespace integrators {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

            class ImplicitTrapezoidal : public FixedStepIntegrator{

                VectorDynSize m_computationBuffer, m_computationBuffer2;
                MatrixDynSize m_identity, m_stateJacBuffer, m_controlJacBuffer;
                MatrixDynSize m_zeroNxNxBuffer, m_zeroNuNuBuffer, m_zeroNxNuBuffer;
                MatrixDynSize m_stateHessianBuffer, m_controlHessianBuffer, m_mixedHessianBuffer;
                VectorDynSize m_lambda;
                bool m_hasStateJacobianSparsity = false;
                bool m_hasControlJacobianSparsity = false;
                std::vector<SparsityStructure> m_stateJacobianSparsity;
                std::vector<SparsityStructure> m_controlJacobianSparsity;
                bool m_hasStateHessianSparsity = false;
                bool m_hasStateControlHessianSparsity = false;
                bool m_hasControlHessianSparsity = false;
                CollocationHessianSparsityMap m_stateHessianSparsity;
                CollocationHessianSparsityMap m_stateControlHessianSparsity;
                CollocationHessianSparsityMap m_controlHessianSparsity;

                virtual bool allocateBuffers() override;

                virtual bool oneStepIntegration(double t0, double dT, const VectorDynSize& x0, VectorDynSize& x) override;

            public:

                ImplicitTrapezoidal();

                ImplicitTrapezoidal(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem);

                virtual ~ImplicitTrapezoidal() override;

                virtual bool evaluateCollocationConstraint(double time, const std::vector<VectorDynSize> &collocationPoints,
                                                           const std::vector<VectorDynSize> &controlInputs, double dT,
                                                           VectorDynSize &constraintValue) override;

                virtual bool evaluateCollocationConstraintJacobian(double time, const std::vector<VectorDynSize> &collocationPoints,
                                                                   const std::vector<VectorDynSize> &controlInputs, double dT,
                                                                   std::vector<MatrixDynSize> &stateJacobianValues,
                                                                   std::vector<MatrixDynSize> &controlJacobianValues) override;

                virtual bool getCollocationConstraintJacobianStateSparsity(std::vector<SparsityStructure>& stateJacobianSparsity) override;

                virtual bool getCollocationConstraintJacobianControlSparsity(std::vector<SparsityStructure>& controlJacobianSparsity) override;

                virtual bool evaluateCollocationConstraintSecondDerivatives(double time, const std::vector<VectorDynSize>& collocationPoints,
                                                                            const std::vector<VectorDynSize>& controlInputs, double dT,
                                                                            const VectorDynSize& lambda,
                                                                            CollocationHessianMap& stateSecondDerivative,
                                                                            CollocationHessianMap& controlSecondDerivative,
                                                                            CollocationHessianMap& stateControlSecondDerivative) override;

                virtual bool getCollocationConstraintSecondDerivativeWRTStateSparsity(CollocationHessianSparsityMap& stateDerivativeSparsity) override;

                virtual bool getCollocationConstraintSecondDerivativeWRTControlSparsity(CollocationHessianSparsityMap& controlDerivativeSparsity) override;

                virtual bool getCollocationConstraintSecondDerivativeWRTStateControlSparsity(CollocationHessianSparsityMap& stateControlDerivativeSparsity) override;

            };

        }
    }
}

#endif // IMPLICITTRAPEZOIDAL_H
