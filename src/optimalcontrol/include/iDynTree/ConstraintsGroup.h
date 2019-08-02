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

#ifndef IDYNTREE_OPTIMALCONTROL_CONSTRAINTSGROUP_H
#define IDYNTREE_OPTIMALCONTROL_CONSTRAINTSGROUP_H

#include <memory>
#include <string>
#include <vector>
#include <iDynTree/SparsityStructure.h>

namespace iDynTree {

    class VectorDynSize;
    class MatrixDynSize;

    namespace optimalcontrol {

        class Constraint;
        class LinearConstraint;
        class TimeRange;

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        /**
         * @brief Class grouping constraints associated with a TimeRange.
         *
         * This class allows to define a set of constraints which are enabled only for a particular time range. This allow to change the constraint structure depending on time. Given a specific time instant, only one constraint is enabled (the one whose TimeRange contains the instant and has the higher initTime).
         * If the specified time instant does not fall into any time constraint time range, a dummy constraint wil be evaluated, i.e. \f$ -1 \leq 0 \leq 1 \f$. All the constraints that will be added to the group should have dimension at most equal to maxConstraintSize.
         * If the constraint size is smaller than the maxConstraintSize, dummy constraints as the above will be added on the bottom. This allow to keep a constant structure even if the constraints have different dimensions.
         * A typical example is when a constraint is enabled only for a certain period of time.
         */
        class ConstraintsGroup{
        public:
            /**
             * @brief Default constructor
             * @param[in] name Univocal name of the group
             * @param[in] maxConstraintSize Maximum dimension allowed when adding a constraint.
             */
            ConstraintsGroup(const std::string& name, unsigned int maxConstraintSize);

            ~ConstraintsGroup();

            ConstraintsGroup(const ConstraintsGroup& other) = delete;

            /**
             * @brief Return the name of the group.
             * @return The univocal name of the constraint group.
             */
            const std::string name() const;

            /**
             * @brief Return the maximum constraint dimension of the group.
             * @return The maximum constraint dimension of the group.
             */
            unsigned int constraintsDimension() const;

            /**
             * @brief Add a constraint to the group
             * @param[in] constraint Shared pointer to the user defined constraint.
             * @param[in] timeRange Time range in which the constraint will be enabled.
             * @return True if successfull. Posible causes of failures are: empty pointer, dimension bigger than maxConstraintSize, invalid TimeRange.
             */
            bool addConstraint(std::shared_ptr<Constraint> constraint, const TimeRange& timeRange);

            /**
             * @brief Add a linear constraint to the group
             * @param[in] linearConstraint Shared pointer to a linear constraint.
             * @param[in] timeRange Time range in which the constraint will be enabled.
             * @return True if successfull. Posible causes of failures are: empty pointer, dimension bigger than maxConstraintSize, invalid TimeRange.
             */
            bool addConstraint(std::shared_ptr<LinearConstraint> linearConstraint, const TimeRange& timeRange);

            /**
             * @brief Update the TimeRange of a previously added constraint.
             * @param[in] name The name of the constraint whose TimeRange has to be updated.
             * @param[in] timeRange The new TimeRange.
             * @return True if successfull. Possible causes of failure: invalid TimeRange, a constraint does not exist with the specified name.
             */
            bool updateTimeRange(const std::string& name, const TimeRange& timeRange);

            /**
             * @brief Remove a previously added constraint.
             *
             * Note: the sparsity pattern is not updated.
             * @param[in] name The name of the constraint that has to be removed
             * @return True if successfull. Possible causes of failure: a constraint does not exist with the specified name.
             */
            bool removeConstraint(const std::string &name);

            /**
             * @brief Get the TimeRange of the specified constraint.
             * @param[in] name The name of the constraint whose TimeRange you are interested in.
             * @param[out] timeRange The requested TimeRange.
             * @return True if successfull. Possible causes of failure: a constraint does not exist with the specified name.
             */
            bool getTimeRange(const std::string& name, TimeRange& timeRange);

            /**
             * @brief Get a vector containing all the TimeRanges of the added constraints.
             * The order corresponds to the one get via the listConstraints method.
             * @warning It may perform dynamic memory allocation when called the first time and when the number of constraint changes.
             * @return A vector containing all the TimeRanges of the added constraints.
             */
            std::vector<TimeRange>& getTimeRanges();

            /**
             * @brief Check whether, at the specified time, the enabled constraint is satisfied.
             * Given the time instant, this method search for the constraint to be enabled according to the specified TimeRange. Then calls the constraint to check the feasibility of the constraint given the specified state and control.
             * @param[in] time The time at which the constraint group is evaluated.
             * @param[in] state The state at which the constraint group is evaluated.
             * @param[in] control The control at which the constraint group is evaluated.
             * @return True if, at the specified time, the enabled constraint is satisfied.
             */
            bool isFeasibilePoint(double time,
                                  const VectorDynSize& state,
                                  const VectorDynSize& control);

            /**
             * @brief Evaluate the constraint which is enabled at the specified time.
             * Given the time instant, this method search for the constraint to be enabled according to the specified TimeRange. Then it evaluates it given the specified state and control.
             * @param[in] time The time at which the constraint group is evaluated.
             * @param[in] state The state at which the constraint group is evaluated.
             * @param[in] control The control at which the constraint group is evaluated.
             * @param[out] constraints The value obtained by evaluating the enbled constraint.
             * @return  True if the evaluation of the enabled constraint was successfull.
             */
            bool evaluateConstraints(double time,
                                     const VectorDynSize& state,
                                     const VectorDynSize& control,
                                     VectorDynSize& constraints);

            /**
             * @brief Get the constraints lower bound.
             * This value depends upon the chosen time instant.
             * @param[in] time Time at which querying the lower bound.
             * @param[out] lowerBound The lowerBound value.
             * @return True if, for the given instant, a lower bound is set. False otherwise.
             */
            bool getLowerBound(double time, VectorDynSize& lowerBound);

            /**
             * @brief Get the constraints upper bound.
             * This value depends upon the chosen time instant.
             * @param[in] time Time at which querying the upper bound.
             * @param[out] upperBound The upperBound value.
             * @return True if, for the given instant, a upper bound is set. False otherwise.
             */
            bool getUpperBound(double time, VectorDynSize& upperBound);

            /**
             * @brief Evaluate the constraint jacobian with respect to the state, given the specified time instant.
             * Given the time instant, this method search for the constraint to be enabled according to the specified TimeRange. Then it evaluates its jacobian given the specified state and control.
             * @param[in] time Time at which the jacobian is evaluted.
             * @param[in] state The state at which the jacobian is evaluated.
             * @param[in] control The control at which the jacobian is evaluated.
             * @param[out] jacobian The output jacobian.
             * @return True if the evaluation of the corresponding method of the enabled constraint return true. False otherwise.
             */
            bool constraintJacobianWRTState(double time,
                                            const VectorDynSize& state,
                                            const VectorDynSize& control,
                                            MatrixDynSize& jacobian);

            /**
             * @brief Evaluate the constraint jacobian with respect to the control, given the specified time instant.
             * Given the time instant, this method search for the constraint to be enabled according to the specified TimeRange. Then it evaluates its jacobian given the specified state and control.
             * @param[in] time Time at which the jacobian is evaluted.
             * @param[in] state The state at which the jacobian is evaluated.
             * @param[in] control The control at which the jacobian is evaluated.
             * @param[out] jacobian The output jacobian.
             * @return True if the evaluation of the corresponding method of the enabled constraint return true. False otherwise.
             */
            bool constraintJacobianWRTControl(double time,
                                              const VectorDynSize& state,
                                              const VectorDynSize& control,
                                              MatrixDynSize& jacobian);

            /**
             * @brief Returns the set of nonzeros elements in terms of row and colun index, in the state jacobian
             * @param stateSparsity Sparsity structure of the partial derivative of the jacobian wrt state variables.
             * @return true if the sparsity is available. False otherwise.
             */
            bool constraintJacobianWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity) const;

            /**
             * @brief Returns the set of nonzeros elements in terms of row and colun index, in the control jacobian
             * @param controlSparsity Sparsity structure of the partial derivative of the jacobian wrt control variables.
             * @return true if the sparsity is available. False otherwise.
             */
            bool constraintJacobianWRTControlSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity) const;

            /**
             * @brief Evaluate constraint second partial derivative wrt the state variables
             *
             * It is the result of \f$\sum \lambda_i \frac{\partial^2 c(t, x, u)}{\partial x^2}\f$
             * @param[in] time The time at which the partial derivative is computed.
             * @param[in] state The state value at which the partial derivative is computed.
             * @param[in] control The control value at which the partial derivative is computed..
             * @param[in] lambda The lagrange multipliers
             * @param[out] hessian The output partial derivative.
             * @return True if successfull, false otherwise (or if not implemented).
             */
            bool constraintSecondPartialDerivativeWRTState(double time,
                                                           const VectorDynSize& state,
                                                           const VectorDynSize& control,
                                                           const VectorDynSize& lambda,
                                                           MatrixDynSize& hessian);


            /**
             * @brief Evaluate constraint second partial derivative wrt the control
             *
             * It is the result of \f$\sum \lambda_i \frac{\partial^2 c(t, x, u)}{\partial u^2}\f$
             * @param[in] time The time at which the partial derivative is computed.
             * @param[in] state The state value at which the partial derivative is computed.
             * @param[in] control The control value at which the partial derivative is computed.
             * @param[in] lambda The lagrange multipliers
             * @param[out] hessian The output partial derivative.
             * @return True if successfull, false otherwise (or if not implemented).
             */
            bool constraintSecondPartialDerivativeWRTControl(double time,
                                                             const VectorDynSize& state,
                                                             const VectorDynSize& control,
                                                             const VectorDynSize& lambda,
                                                             MatrixDynSize& hessian);


            /**
             * @brief Evaluate constraint second partial derivative wrt the state and control
             *
             * It is the result of \f$\sum \lambda_i \frac{\partial^2 c(t, x, u)}{\partial x \partial u}\f$,
             * thus it has number of rows equals to the number of states and number of cols equal to the number of control inputs.
             * @param[in] time The time at which the partial derivative is computed.
             * @param[in] state The state value at which the partial derivative is computed.
             * @param[in] control The control value at which the partial derivative is computed.
             * @param[in] lambda The lagrange multipliers
             * @param[out] hessian The output partial derivative.
             * @return True if successfull, false otherwise (or if not implemented).
             */
            bool constraintSecondPartialDerivativeWRTStateControl(double time,
                                                                  const VectorDynSize& state,
                                                                  const VectorDynSize& control,
                                                                  const VectorDynSize& lambda,
                                                                  MatrixDynSize& hessian);

            /**
             * @brief Returns the set of nonzeros elements in terms of row and colun index, in the state hessian
             *
             * @warning No check is performed in the indeces. They need to be in the range [0, stateDimension) and [0, stateDimension) respectively.
             * @param stateSparsity Sparsity structure of the partial derivative of the jacobian wrt state variables.
             * @return true if the sparsity is available. False otherwise.
             */
            bool constraintsSecondPartialDerivativeWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity);

            /**
             * @brief Returns the set of nonzeros elements in terms of row and colun index, in the mixed hessian
             *
             * @warning No check is performed in the indeces. They need to be in the range [0, stateDimension) and [0, controlDimension) respectively.
             * @param stateControlSparsity Sparsity structure of the partial derivative of the jacobian wrt state and control variables.
             * @return true if the sparsity is available. False otherwise.
             */
            bool constraintsSecondPartialDerivativeWRTStateControlSparsity(iDynTree::optimalcontrol::SparsityStructure& stateControlSparsity);

            /**
             * @brief Returns the set of nonzeros elements in terms of row and colun index, in the control hessian
             *
             * @warning No check is performed in the indeces. They need to be in the range [0, constraintDimension) and [0, controlDimension) respectively.
             * @param controlSparsity Sparsity structure of the partial derivative of the jacobian wrt control variables.
             * @return true if the sparsity is available. False otherwise.
             */
            bool constraintsSecondPartialDerivativeWRTControlSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity);

            /**
             * @brief Flag returning true if the group is an "AnyTime" group.
             * An "AnyTime" group contains only one constraint which is always enabled. It corresponds to a simple constraint.
             * @return true if the group contains a single constraint always enables. False otherwise.
             */
            bool isAnyTimeGroup();

            /**
             * @brief Number of constraints added in the group
             * @return The number of constraints currently loaded in the group.
             */
            unsigned int numberOfConstraints() const;

            /**
             * @brief Lists the available constraints.
             * @warning Perform memory allocation while creating the output vector.
             * @return The list of the names of the constraints added to the group.
             */
            const std::vector<std::string> listConstraints() const;

            /**
             * @brief Tells if the groups contains only linear constraints.
             * @return true if contains only linear constraints. False if at least one generic constraint is added.
             */
            bool isLinearGroup() const;

        private:
            class ConstraintsGroupPimpl;
            ConstraintsGroupPimpl* m_pimpl;
        };
    }
}

#endif // IDYNTREE_OPTIMALCONTROL_CONSTRAINTSGROUP_H
