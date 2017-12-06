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
#include <vector>
#include <memory>

namespace iDynTree {

namespace optimalcontrol {

        class DynamicalSystem;

        namespace integrators {

            class solutionElement{
            public:
                VectorDynSize stateAtT;
                double time;
            };

            class Integrator {

            public:
                Integrator(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem);

                ~Integrator();

                virtual bool integrate(double initialTime, double finalTime) = 0;

                bool setMaximumStepSize(const double dT);//maybe I should define an empty interface and an abstarct class to implement these basic methods

                double maximumStepSize() const;

                const std::weak_ptr<const DynamicalSystem> dynamicalSystem() const;

                virtual bool getSolution(double time, VectorDynSize& solution) const;

                virtual const std::vector<solutionElement>& getFullSolution() const;

                virtual void clearSolution();

            protected:
                double m_dTmax;
                std::shared_ptr<DynamicalSystem> m_dynamicalSystem_ptr;
                std::vector<solutionElement> m_solution;

                bool interpolatePoints(const std::vector<solutionElement>::const_iterator &first,
                                       const std::vector<solutionElement>::const_iterator &second,
                                       double time, VectorDynSize& outputPoint) const;
            };
        }

    }
}



#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_INTEGRATOR_H */
