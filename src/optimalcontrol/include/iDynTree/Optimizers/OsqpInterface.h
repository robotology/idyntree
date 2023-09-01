// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_OSQPINTERFACE_H
#define IDYNTREE_OPTIMALCONTROL_OSQPINTERFACE_H

#include <memory>
#include <string>
#include <iDynTree/Optimizer.h>

namespace iDynTree {

    class VectorDynSize;

    namespace optimization {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        typedef struct {
            double rho = 0.1; //>0
            double sigma = 1e-06; //>0, when changed prevent reOptimize
            unsigned int max_iter = 4000; // >0
            double eps_abs = 1e-03; //>=0
            double eps_rel = 1e-03; //>=0
            double eps_prim_inf = 1e-04; //>=0
            double eps_dual_inf  = 1e-04; //>=0
            double alpha = 1.6; // 0 < x < 2
            unsigned int linsys_solver = 0; //0/1, when changed prevent reOptimize
            double delta = 1e-06; //<0
            bool polish = false;
            unsigned int polish_refine_iter = 3; //>0
            bool verbose = true;
            bool scaled_termination = false;
            unsigned int check_termination = 25;
            bool warm_start = true;
            unsigned int scaling = 10; //when changed prevent reOptimize
            bool adaptive_rho = true; // when changed prevent reOptimize
            unsigned int adaptive_rho_interval = 0; // when changed prevent reOptimize
            double adaptive_rho_tolerance = 5; //>=1, when changed prevent reOptimize
            double adaptive_rho_fraction = 0.4; //>0// when changed prevent reOptimize
            double time_limit = 0; //>=0
        } OsqpSettings;

        class OsqpInterface : public Optimizer {

            class OsqpInterfaceImplementation;
            OsqpInterfaceImplementation *m_pimpl;

        public:

            OsqpInterface();

            OsqpInterface(const OsqpInterface &other) = delete;

            virtual ~OsqpInterface() override;

            virtual bool isAvailable() const override;

            virtual bool setProblem(std::shared_ptr<OptimizationProblem> problem) override;

            virtual bool solve() override;

            virtual bool getPrimalVariables(VectorDynSize &primalVariables) override;

            virtual bool getDualVariables(VectorDynSize &constraintsMultipliers,
                                          VectorDynSize &lowerBoundsMultipliers,
                                          VectorDynSize &upperBoundsMultipliers) override;

            virtual bool getOptimalCost(double &optimalCost) override;

            virtual bool getOptimalConstraintsValues(VectorDynSize &constraintsValues) override;

            virtual double minusInfinity() override;

            virtual double plusInfinity() override;

            OsqpSettings& settings();
        };

    }

}

#endif // IDYNTREE_OPTIMALCONTROL_OSQPINTERFACE_H
