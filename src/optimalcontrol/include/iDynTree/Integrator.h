/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_INTEGRATOR_H
#define IDYNTREE_OPTIMALCONTROL_INTEGRATOR_H

#include "iDynTree/Core/VectorDynSize.h"
#include "iDynTree/Core/MatrixDynSize.h"
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

            class solutionElement{
            public:
                VectorDynSize stateAtT;
                double time;
            };


            class IntegratorInfoData {
            protected:
                friend class Integrator;
                IntegratorInfoData():name("Integrator"),isExplicit(true),numberOfStages(0){}
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

                IntegratorInfo(const IntegratorInfo &other) = delete;

                const std::string &name() const {return m_data->name;}

                const bool isExplicit() const {return m_data->isExplicit;}

                const size_t numberOfStages() const {return m_data->numberOfStages;}
            };

            /**
             * @warning This class is still in active development, and so API interface can change between iDynTree versions.
             * \ingroup iDynTreeExperimental
             */

            class Integrator {

            public:
                Integrator(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem);

                ~Integrator();

                virtual bool integrate(double initialTime, double finalTime) = 0;

                bool setMaximumStepSize(const double dT);

                double maximumStepSize() const;

                const std::weak_ptr<const DynamicalSystem> dynamicalSystem() const;

                virtual bool getSolution(double time, VectorDynSize& solution) const;

                virtual const std::vector<solutionElement>& getFullSolution() const;

                virtual void clearSolution();

                virtual bool evaluateCollocationConstraint(const std::vector<VectorDynSize>& collocationPoints, double& constraintValue);

                virtual bool evaluateCollocationConstraintJacobian(const std::vector<VectorDynSize>& collocationPoints, MatrixDynSize& jacobianValue);

                const IntegratorInfo& info() const;

            protected:
                double m_dTmax;
                std::shared_ptr<DynamicalSystem> m_dynamicalSystem_ptr;
                std::vector<solutionElement> m_solution;
                std::shared_ptr<IntegratorInfoData> m_infoData;
                IntegratorInfo m_info;

                virtual bool interpolatePoints(const std::vector<solutionElement>::const_iterator &first,
                                               const std::vector<solutionElement>::const_iterator &second,
                                               double time, VectorDynSize& outputPoint) const;
            };
        }

    }
}



#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_INTEGRATOR_H */
