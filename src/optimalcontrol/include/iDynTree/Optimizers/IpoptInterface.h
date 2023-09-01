// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_IPOPTINTERFACE_H
#define IDYNTREE_OPTIMALCONTROL_IPOPTINTERFACE_H

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

        class IpoptInterface : public Optimizer {

            class IpoptInterfaceImplementation;
            IpoptInterfaceImplementation *m_pimpl;

        public:

            IpoptInterface();

            IpoptInterface(const IpoptInterface &other) = delete;

            virtual ~IpoptInterface() override;

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

            bool setIpoptOption(const std::string &tag, const std::string &value);

            bool setIpoptOption(const std::string &tag, double value);

            bool setIpoptOption(const std::string &tag, int value);

            bool getIpoptOption(const std::string &tag, std::string &value);

            bool getIpoptOption(const std::string &tag, double &value);

            bool getIpoptOption(const std::string &tag, int &value);
        };

    }

}

#endif // IDYNTREE_OPTIMALCONTROL_IPOPTINTERFACE_H
