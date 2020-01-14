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

#ifndef IDYNTREE_OPTIMALCONTROL_WORHPINTERFACE_H
#define IDYNTREE_OPTIMALCONTROL_WORHPINTERFACE_H

#include <iDynTree/Optimizer.h>

#include <memory>
#include <string>

namespace iDynTree {

    class VectorDynSize;

    namespace optimization {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class WorhpInterface : public Optimizer {

            class WorhpInterfaceImplementation;
            WorhpInterfaceImplementation *m_pimpl;

        public:

            WorhpInterface();

            WorhpInterface(const WorhpInterface &other) = delete;

            virtual ~WorhpInterface() override;

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

            void useApproximatedHessians(bool useApproximatedHessian = true);

            bool setWorhpParam(const std::string &paramName, bool value);

            bool setWorhpParam(const std::string &paramName, double value);

            bool setWorhpParam(const std::string &paramName, int value);

            bool getWorhpParam(const std::string &paramName, bool &value);

            bool getWorhpParam(const std::string &paramName, double &value);

            bool getWorhpParam(const std::string &paramName, int &value);
        };
    }
}

#endif // IDYNTREE_OPTIMALCONTROL_WORHPINTERFACE_H
