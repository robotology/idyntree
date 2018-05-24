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

#ifndef IDYNTREE_OPTIMALCONTROL_INTEGRATOR_H
#define IDYNTREE_OPTIMALCONTROL_INTEGRATOR_H

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <vector>
#include <string>
#include <memory>

namespace iDynTree {

namespace optimalcontrol {

        class DynamicalSystem;

        namespace integrators {

            /**
             * @warning This class is still in active development, and so API interface can change between iDynTree versions.
             * \ingroup iDynTreeExperimental
             */

            class SolutionElement{
            public:
                VectorDynSize stateAtT;
                double time;
            };


            class IntegratorInfoData {
            protected:
                friend class Integrator;
                IntegratorInfoData():name("Integrator"),isExplicit(true),numberOfStages(1){}
            public:
                std::string name;

                bool isExplicit;

                size_t numberOfStages;
            };

            class IntegratorInfo {
            private:
                std::shared_ptr<IntegratorInfoData> m_data;
            public:
                IntegratorInfo(std::shared_ptr<IntegratorInfoData> data):m_data(data){}

                IntegratorInfo() = delete;

                IntegratorInfo(const IntegratorInfo &other) = delete;

                const std::string &name() const {return m_data->name;}

                 bool isExplicit() const {return m_data->isExplicit;}

                 size_t numberOfStages() const {return m_data->numberOfStages;}
            };

            /**
             * @warning This class is still in active development, and so API interface can change between iDynTree versions.
             * \ingroup iDynTreeExperimental
             */

            class Integrator {

            public:

                Integrator();

                Integrator(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem);

                virtual ~Integrator();

                virtual bool integrate(double initialTime, double finalTime) = 0;

                bool setMaximumStepSize(const double dT);

                double maximumStepSize() const;

                bool setDynamicalSystem(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem);

                const std::weak_ptr<DynamicalSystem> dynamicalSystem() const;

                virtual bool getSolution(double time, VectorDynSize& solution) const;

                virtual const std::vector<SolutionElement>& getFullSolution() const;

                virtual void clearSolution();

                virtual bool evaluateCollocationConstraint(double time, const std::vector<VectorDynSize>& collocationPoints,
                                                           const std::vector<VectorDynSize>& controlInputs, double dT,
                                                           VectorDynSize& constraintValue);

                virtual bool evaluateCollocationConstraintJacobian(double time, const std::vector<VectorDynSize>& collocationPoints,
                                                                   const std::vector<VectorDynSize>& controlInputs, double dT,
                                                                   std::vector<MatrixDynSize>& stateJacobianValues,
                                                                   std::vector<MatrixDynSize>& controlJacobianValues);

                const IntegratorInfo& info() const;

            protected:
                double m_dTmax;
                std::shared_ptr<DynamicalSystem> m_dynamicalSystem_ptr;
                std::vector<SolutionElement> m_solution;
                std::shared_ptr<IntegratorInfoData> m_infoData;
                IntegratorInfo m_info;

                virtual bool interpolatePoints(const std::vector<SolutionElement>::const_iterator &first,
                                               const std::vector<SolutionElement>::const_iterator &second,
                                               double time, VectorDynSize& outputPoint) const;

                virtual bool allocateBuffers();
            };
        }

    }
}



#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_INTEGRATOR_H */
