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

#ifndef IDYNTREE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H
#define IDYNTREE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H

#include <memory>
#include <vector>
#include <string>

#include <iDynTree/Core/Utils.h>
#include <iDynTree/SparsityStructure.h>
#include <iDynTree/TimeVaryingObject.h>


namespace iDynTree {

    class VectorDynSize;
    class MatrixDynSize;

    namespace optimalcontrol {

        class DynamicalSystem;
        class LinearSystem;
        class Constraint;
        class LinearConstraint;
        class ConstraintsGroup;
        class Cost;
        class QuadraticCost;
        class L2NormCost;
        class LinearCost;
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

            bool setDynamicalSystemConstraint(std::shared_ptr<LinearSystem> linearSystem);

            const std::weak_ptr<DynamicalSystem> dynamicalSystem() const;

            bool systemIsLinear() const;

            bool addGroupOfConstraints(std::shared_ptr<ConstraintsGroup> groupOfConstraints); //to be used when the constraints applies only for a time interval

            bool removeGroupOfConstraints(const std::string& name);

            bool addConstraint(std::shared_ptr<Constraint> newConstraint); // this apply for the full horizon. It creates a group named with the same name and containing only newConstraint

            bool addConstraint(std::shared_ptr<LinearConstraint> newConstraint);

            bool removeConstraint(const std::string& name); //this removes only the constraints added with the above method

            unsigned int countConstraints() const;

            unsigned int countLinearConstraints() const;

            unsigned int getConstraintsDimension() const;

            const std::vector<std::string> listConstraints() const;

            const std::vector<std::string> listGroups() const; //the i-th entry of the list contains the i-th constraint displayed with listConstraints()

            std::vector<TimeRange>& getConstraintsTimeRanges() const;

            std::vector<size_t>& getLinearConstraintsIndeces() const;

            // Cost can be:
            // Mayer term
            // Lagrange term
            bool addMayerTerm(double weight, std::shared_ptr<Cost> cost); // final cost

            bool addMayerTerm(double weight, std::shared_ptr<QuadraticCost> quadraticCost); // final cost

            bool addMayerTerm(double weight, std::shared_ptr<L2NormCost> quadraticCost); // final cost

            bool addMayerTerm(double weight, std::shared_ptr<LinearCost> linearCost); // final cost

            bool addLagrangeTerm(double weight, std::shared_ptr<Cost> cost); // integral cost

            bool addLagrangeTerm(double weight, std::shared_ptr<QuadraticCost> quadraticCost); // integral cost

            bool addLagrangeTerm(double weight, std::shared_ptr<L2NormCost> quadraticCost); // integral cost

            bool addLagrangeTerm(double weight, std::shared_ptr<LinearCost> linearCost); // integral cost

            bool addLagrangeTerm(double weight,
                                 double startingTime,
                                 double finalTime,
                                 std::shared_ptr<Cost> cost); // integral cost with explicit integration limits

            bool addLagrangeTerm(double weight,
                                 double startingTime,
                                 double finalTime,
                                 std::shared_ptr<QuadraticCost> quadraticCost); // integral cost with explicit integration limits

            bool addLagrangeTerm(double weight,
                                 double startingTime,
                                 double finalTime,
                                 std::shared_ptr<L2NormCost> quadraticCost); // integral cost with explicit integration limits

            bool addLagrangeTerm(double weight,
                                 double startingTime,
                                 double finalTime,
                                 std::shared_ptr<LinearCost> linearCost); // integral cost with explicit integration limits

            bool addLagrangeTerm(double weight, const TimeRange& timeRange, std::shared_ptr<Cost> cost);

            bool addLagrangeTerm(double weight, const TimeRange& timeRange, std::shared_ptr<QuadraticCost> quadraticCost);

            bool addLagrangeTerm(double weight, const TimeRange& timeRange, std::shared_ptr<L2NormCost> quadraticCost);

            bool addLagrangeTerm(double weight, const TimeRange& timeRange, std::shared_ptr<LinearCost> linearCost);

            bool updateCostTimeRange(const std::string& name, double newStartingTime, double newEndTime);

            bool updateCostTimeRange(const std::string& name, const TimeRange& newTimeRange);

            bool removeCost(const std::string& name);

            std::vector<TimeRange>& getCostsTimeRanges() const;

            bool hasOnlyLinearCosts() const;

            bool hasOnlyQuadraticCosts() const;

            bool setStateLowerBound(const VectorDynSize& minState);

            bool setStateLowerBound(std::shared_ptr<TimeVaryingVector> minState);

            bool setStateUpperBound(const VectorDynSize& maxState);

            bool setStateUpperBound(std::shared_ptr<TimeVaryingVector> maxState);

            bool setControlLowerBound(const VectorDynSize& minControl);

            bool setControlLowerBound(std::shared_ptr<TimeVaryingVector> minControl);

            bool setControlUpperBound(const VectorDynSize& maxControl);

            bool setControlUpperBound(std::shared_ptr<TimeVaryingVector> maxControl);

            bool setStateBoxConstraints(const VectorDynSize& minState,
                                        const VectorDynSize& maxState);            

            bool setStateBoxConstraints(std::shared_ptr<TimeVaryingVector> minState,
                                        std::shared_ptr<TimeVaryingVector> maxState);

            bool setControlBoxConstraints(const VectorDynSize& minControl,
                                          const VectorDynSize& maxControl);

            bool setControlBoxConstraints(std::shared_ptr<TimeVaryingVector> minControl,
                                          std::shared_ptr<TimeVaryingVector> maxControl);

            bool getStateLowerBound(double time, VectorDynSize& minState);  //return false if the corresponding bound is not set

            bool getStateUpperBound(double time, VectorDynSize& maxState);  //return false if the corresponding bound is not set

            bool getControlLowerBound(double time, VectorDynSize& minControl);  //return false if the corresponding bound is not set

            bool getControlUpperBound(double time, VectorDynSize& maxControl);  //return false if the corresponding bound is not set

            bool costsEvaluation(double time, const VectorDynSize& state, const VectorDynSize& control, double& costValue);

            bool costsFirstPartialDerivativeWRTState(double time,
                                                     const VectorDynSize& state,
                                                     const VectorDynSize& control,
                                                     VectorDynSize& partialDerivative);

            bool costsFirstPartialDerivativeWRTControl(double time,
                                                       const VectorDynSize& state,
                                                       const VectorDynSize& control,
                                                       VectorDynSize& partialDerivative);

            bool costsSecondPartialDerivativeWRTState(double time,
                                                      const VectorDynSize& state,
                                                      const VectorDynSize& control,
                                                      MatrixDynSize& partialDerivative);

            bool costsSecondPartialDerivativeWRTControl(double time,
                                                        const VectorDynSize& state,
                                                        const VectorDynSize& control,
                                                        MatrixDynSize& partialDerivative);


            bool costsSecondPartialDerivativeWRTStateControl(double time,
                                                             const VectorDynSize& state,
                                                             const VectorDynSize& control,
                                                             MatrixDynSize& partialDerivative);

            bool costsSecondPartialDerivativeWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity); //returns false if something goes wrong

            bool costsSecondPartialDerivativeWRTControlSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity); //returns false if something goes wrong

            bool costsSecondPartialDerivativeWRTStateControlSparsity(iDynTree::optimalcontrol::SparsityStructure& stateControlSparsity); //returns false if something goes wrong

            bool constraintsEvaluation(double time, const VectorDynSize& state, const VectorDynSize& control, VectorDynSize& constraintsValue);

            bool getConstraintsUpperBound(double time, double infinity, VectorDynSize& upperBound); //returns false if something goes wrong

            bool getConstraintsLowerBound(double time, double infinity, VectorDynSize& lowerBound); //returns false if something goes wrong

            bool isFeasiblePoint(double time, const VectorDynSize& state, const VectorDynSize& control);

            bool constraintsJacobianWRTState(double time,
                                             const VectorDynSize& state,
                                             const VectorDynSize& control,
                                             MatrixDynSize& jacobian);

            bool constraintsJacobianWRTControl(double time,
                                               const VectorDynSize& state,
                                               const VectorDynSize& control,
                                               MatrixDynSize& jacobian);

            bool constraintsJacobianWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity); //returns false if something goes wrong

            bool constraintsJacobianWRTControlSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity); //returns false if something goes wrong

            bool constraintsSecondPartialDerivativeWRTState(double time,
                                                           const VectorDynSize& state,
                                                           const VectorDynSize& control,
                                                           const VectorDynSize& lambda,
                                                           MatrixDynSize& hessian);

            bool constraintsSecondPartialDerivativeWRTControl(double time,
                                                             const VectorDynSize& state,
                                                             const VectorDynSize& control,
                                                             const VectorDynSize& lambda,
                                                             MatrixDynSize& hessian);

            bool constraintsSecondPartialDerivativeWRTStateControl(double time,
                                                                  const VectorDynSize& state,
                                                                  const VectorDynSize& control,
                                                                  const VectorDynSize& lambda,
                                                                  MatrixDynSize& hessian);

            bool constraintsSecondPartialDerivativeWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity); //returns false if something goes wrong

            bool constraintsSecondPartialDerivativeWRTControlSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity); //returns false if something goes wrong

            bool constraintsSecondPartialDerivativeWRTStateControlSparsity(iDynTree::optimalcontrol::SparsityStructure& stateControlSparsity); //returns false if something goes wrong

        private:
            class OptimalControlProblemPimpl;
            OptimalControlProblemPimpl* m_pimpl;
            
        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H */
