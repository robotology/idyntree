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

#ifndef IDYNTREE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H
#define IDYNTREE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H

#include <memory>
#include <vector>
#include <string>

namespace iDynTree {

    class VectorDynSize;
    class MatrixDynSize;

    namespace optimalcontrol {

        class DynamicalSystem;
        class Constraint;
        class ConstraintsGroup;
        class Cost;
        class TimeRange;

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class OptimalControlProblem {
        public:

            OptimalControlProblem();

            ~OptimalControlProblem();

            OptimalControlProblem(const OptimalControlProblem& other) = delete;

            bool setTimeHorizon(double startingTime, double finalTime);
            double initialTime() const;
            double finalTime() const;

            bool setDynamicalSystemConstraint(std::shared_ptr<DynamicalSystem> dynamicalSystem);
            const std::weak_ptr<const iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem() const;

            bool addGroupOfConstraints(std::shared_ptr<ConstraintsGroup> groupOfConstraints); //to be used when the constraints applies only for a time interval

            bool removeGroupOfConstraints(const std::string& name);

            bool addContraint(std::shared_ptr<Constraint> newConstraint); // this apply for the full horizon. It creates a group named with the same name and containing only newConstraint
            bool removeConstraint(const std::string& name); //this removes only the constraints added with the above method
            unsigned int countConstraints() const;
            unsigned int getConstraintsDimension() const;
            const std::vector<std::string> listConstraints() const;
            const std::vector<std::string> listGroups() const; //the i-th entry of the list contains the i-th constraint displayed with listConstraints()

            // Cost can be:
            // Mayer term
            // Lagrange term
            bool addMayerTerm(double weight, std::shared_ptr<Cost> cost); // final cost

            bool addLagrangeTerm(double weight, std::shared_ptr<Cost> cost); // integral cost

            bool addLagrangeTerm(double weight,
                                 double startingTime,
                                 double finalTime,
                                 std::shared_ptr<Cost> cost); // integral cost with explicit integration limits

            bool addLagrangeTerm(double weight, const TimeRange& timeRange, std::shared_ptr<Cost> cost);

            bool updateCostTimeRange(const std::string& name, double newStartingTime, double newEndTime);

            bool updateCostTimeRange(const std::string& name, const TimeRange& newTimeRange);

            bool removeCost(const std::string& name);

            bool setStateBoxConstraints(const iDynTree::VectorDynSize& minState,
                                        const iDynTree::VectorDynSize& maxState);
            bool setControlBoxConstraints(const iDynTree::VectorDynSize& minControl,
                                          const iDynTree::VectorDynSize& maxControl);

            bool costsEvaluation(double time, const iDynTree::VectorDynSize& state, const iDynTree::VectorDynSize& control, double& costValue);

            bool costsFirstPartialDerivativeWRTState(double time,
                                                     const iDynTree::VectorDynSize& state,
                                                     const iDynTree::VectorDynSize& control,
                                                     iDynTree::VectorDynSize& partialDerivative);

            bool costFirstPartialDerivativeWRTControl(double time,
                                                      const iDynTree::VectorDynSize& state,
                                                      const iDynTree::VectorDynSize& control,
                                                      iDynTree::VectorDynSize& partialDerivative);

            bool costSecondPartialDerivativeWRTState(double time,
                                                     const iDynTree::VectorDynSize& state,
                                                     const iDynTree::VectorDynSize& control,
                                                     iDynTree::MatrixDynSize& partialDerivative);

            bool costSecondPartialDerivativeWRTControl(double time,
                                                       const iDynTree::VectorDynSize& state,
                                                       const iDynTree::VectorDynSize& control,
                                                       iDynTree::MatrixDynSize& partialDerivative);


            bool costSecondPartialDerivativeWRTStateControl(double time,
                                                            const iDynTree::VectorDynSize& state,
                                                            const iDynTree::VectorDynSize& control,
                                                            iDynTree::MatrixDynSize& partialDerivative);

            bool constraintsEvaluation(double time, const iDynTree::VectorDynSize& state, const iDynTree::VectorDynSize& control, iDynTree::VectorDynSize& constraintsValue);

            bool isFeasiblePoint(double time, const iDynTree::VectorDynSize& state, const iDynTree::VectorDynSize& control);

            bool constraintsJacobianWRTState(double time,
                                             const VectorDynSize& state,
                                             const VectorDynSize& control,
                                             MatrixDynSize& jacobian);

            bool constraintsJacobianWRTControl(double time,
                                               const VectorDynSize& state,
                                               const VectorDynSize& control,
                                               MatrixDynSize& jacobian);

        private:
            class OptimalControlProblemPimpl;
            OptimalControlProblemPimpl* m_pimpl;
            
        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H */
