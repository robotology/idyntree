/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_RK4_H
#define IDYNTREE_OPTIMALCONTROL_RK4_H

#include "iDynTree/Integrator.h"
#include "Eigen/Sparse"
#include "Eigen/Dense"
#include "iDynTree/Core/VectorDynSize.h"

namespace iDynTree {
    namespace optimalcontrol {

        class DynamicalSystem;

        namespace integrators {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

            class RK4 : public Integrator
            {
                Eigen::SparseMatrix<double> m_aCoefficents;
                Eigen::VectorXd m_bCoefficients;
                Eigen::VectorXd m_cCoefficients;
                Eigen::MatrixXd m_K;
                VectorDynSize m_computationBuffer;

                bool oneStepIntegration(double t0, double dT, const VectorDynSize& x0, VectorDynSize& x);

            public:
                RK4(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem);

                ~RK4();

                bool integrate(double initialTime, double finalTime) override;

            };
        }
    }
}

#endif // RK4_H
