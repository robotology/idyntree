// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_ALGLIBINTERFACE_H
#define IDYNTREE_OPTIMALCONTROL_ALGLIBINTERFACE_H

#include <iDynTree/Optimizer.h>

namespace iDynTree {

    class VectorDynSize;

    namespace optimization {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class AlglibInterface : public Optimizer {

            class AlglibInterfaceImplementation;
            AlglibInterfaceImplementation *m_pimpl;

        public:

            AlglibInterface();

            AlglibInterface(const AlglibInterface &other) = delete;

            virtual ~AlglibInterface() override;

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

            //Set the penalty coefficient that multiplies constraints value in the cost function of the correspondent uncostrained problem.
            // See http://www.alglib.net/translator/man/manual.cpp.html#sub_minnlcsetalgoaul
            bool setRHO(double rho);

            //Number of iterations to refine Lagrange multipliers.
            //See http://www.alglib.net/translator/man/manual.cpp.html#sub_minnlcsetalgoaul
            bool setOuterIterations(unsigned int outerIterations);

            //epsX: stopping criterion on the variation of optimization variables
            //See http://www.alglib.net/translator/man/manual.cpp.html#sub_minnlcsetcond
            bool setStoppingCondition(double epsX);

            bool setMaximumIterations(unsigned int maxIter);

        };

    }

}

#endif // ALGLIBINTERFACE_H
