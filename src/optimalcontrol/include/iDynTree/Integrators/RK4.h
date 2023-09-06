// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_RK4_H
#define IDYNTREE_OPTIMALCONTROL_RK4_H

#include <iDynTree/Integrators/FixedStepIntegrator.h>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <iDynTree/VectorDynSize.h>

namespace iDynTree {
    namespace optimalcontrol {

        class DynamicalSystem;

        namespace integrators {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

            class RK4 : public FixedStepIntegrator
            {
                Eigen::SparseMatrix<double> m_aCoefficents;
                Eigen::VectorXd m_bCoefficients;
                Eigen::VectorXd m_cCoefficients;
                Eigen::MatrixXd m_K;
                VectorDynSize m_computationBuffer;

                bool oneStepIntegration(double t0, double dT, const VectorDynSize& x0, VectorDynSize& x) override;

                bool allocateBuffers() override;

            public:
                RK4();

                RK4(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem);

                virtual ~RK4();
            };
        }
    }
}

#endif // RK4_H
