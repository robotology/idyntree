// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/VectorDynSize.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/Utils.h>
#include <iDynTree/ConstraintsGroup.h>
#include <iDynTree/Constraint.h>
#include <iDynTree/LinearConstraint.h>
#include <iDynTree/TimeRange.h>
#include <iDynTree/EigenHelpers.h>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cassert>
#include <sstream>

namespace iDynTree{
namespace optimalcontrol {

        typedef struct{
            std::shared_ptr<Constraint> constraint;
            TimeRange timeRange;
            VectorDynSize constraintBuffer;
            MatrixDynSize stateJacobianBuffer;
            MatrixDynSize controlJacobianBuffer;
            VectorDynSize lambdaBuffer;
        }TimedConstraint;

        typedef std::shared_ptr<TimedConstraint> TimedConstraint_ptr;

        typedef std::unordered_map<std::string, TimedConstraint_ptr> GroupOfConstraintsMap;

        class ConstraintsGroup::ConstraintsGroupPimpl{
        public:
            GroupOfConstraintsMap group;
            std::vector<TimedConstraint_ptr> orderedIntervals;
            SparsityStructure groupStateJacobianSparsity;
            SparsityStructure groupControlJacobianSparsity;
            SparsityStructure groupStateHessianSparsity;
            SparsityStructure groupControlHessianSparsity;
            SparsityStructure groupMixedHessianSparsity;
            std::string name;
            unsigned int maxConstraintSize;
            std::vector<TimeRange> timeRanges;
            bool isLinearGroup = true;
            bool stateJacobianSparsityProvided = true;
            bool controlJacobianSparsityProvided = true;
            bool stateHessianSparsityProvided = true;
            bool controlHessianSparsityProvided = true;
            bool mixedHessianSparsityProvided = true;


            std::vector<TimedConstraint_ptr>::reverse_iterator findActiveConstraint(double time){
                return std::find_if(orderedIntervals.rbegin(),
                                    orderedIntervals.rend(),
                                    [time](const TimedConstraint_ptr & a) -> bool { return a->timeRange.isInRange(time); }); //find the last element in the vector with init time lower than the specified time
            }

            bool addConstraint(std::shared_ptr<Constraint> constraint, const TimeRange &timeRange)
            {
                if (!constraint){
                    reportError("ConstraintsGroup", "addConstraint", "Empty constraint pointer.");
                    return false;
                }

                if (constraint->constraintSize() > maxConstraintSize){
                    reportError("ConstraintsGroup", "addConstraint", "The constraint dimension is greater than the maximum allowed by the group.");
                    return false;
                }

                if (timeRange == TimeRange::AnyTime()){
                    if (group.size() != 0){
                        reportError("ConstraintsGroup", "addConstraint",
                                    "Only one constraint is allowed in a group if the timeRange is AnyTime.");
                        return false;
                    }
                } else {
                    if (!timeRange.isValid()){
                        reportError("ConstraintsGroup", "addConstraint", "Invalid timeRange.");
                        return false;
                    }
                }

                if(group.size() > 0){
                    GroupOfConstraintsMap::iterator constraintIterator;
                    constraintIterator = group.find(constraint->name());
                    if(constraintIterator != group.end()){
                        std::ostringstream errorMsg;
                        errorMsg << "A constraint named " << constraint->name()
                                 <<" already exists in the group "<< name << " .";
                        reportError("ConstraintsGroup", "addConstraint", errorMsg.str().c_str());
                        return false;
                    }
                }

                //add constraints in the group
                TimedConstraint_ptr newConstraint = std::make_shared<TimedConstraint>();

                newConstraint->timeRange = timeRange;
                newConstraint->constraint = constraint;
                newConstraint->constraintBuffer.resize(static_cast<unsigned int>(constraint->constraintSize()));
                newConstraint->stateJacobianBuffer.resize(static_cast<unsigned int>(constraint->constraintSize()), static_cast<unsigned int>(constraint->expectedStateSpaceSize()));
                newConstraint->controlJacobianBuffer.resize(static_cast<unsigned int>(constraint->constraintSize()), static_cast<unsigned int>(constraint->expectedControlSpaceSize()));
                newConstraint->lambdaBuffer.resize(static_cast<unsigned int>(constraint->constraintSize()));


                std::pair< GroupOfConstraintsMap::iterator, bool> result;
                result = group.insert(GroupOfConstraintsMap::value_type(constraint->name(), newConstraint));

                if(!result.second){
                    std::ostringstream errorMsg;
                    errorMsg << "Unable to add constraint "<<constraint->name() << std::endl;
                    reportError("ConstraintsGroup", "addConstraint", errorMsg.str().c_str());
                    return false;
                }

                orderedIntervals.push_back(result.first->second); //register the time range in order to have the constraints ordered by init time. result.first->second is the TimedConstraint_ptr of the newly inserted TimedConstraint.
                std::sort(orderedIntervals.begin(), orderedIntervals.end(), [](const TimedConstraint_ptr&a, const TimedConstraint_ptr&b) { return a->timeRange < b->timeRange;}); //reorder the vector

                {
                    SparsityStructure newConstraintStateJacobianSparsity, newConstraintControlJacobianSparsity;

                    if (stateJacobianSparsityProvided
                        && constraint->constraintJacobianWRTStateSparsity(newConstraintStateJacobianSparsity)) {

                        if (!newConstraintStateJacobianSparsity.isValid()) {
                            reportError("ConstraintsGroup", "addConstraint", "The state sparsity is provided but the dimension of the two vectors do not match.");
                            return false;
                        }

                        groupStateJacobianSparsity.merge(newConstraintStateJacobianSparsity);

                    } else {
                        stateJacobianSparsityProvided = false;
                    }

                    if (controlJacobianSparsityProvided
                        && constraint->constraintJacobianWRTControlSparsity(newConstraintControlJacobianSparsity)) {

                        if (!newConstraintControlJacobianSparsity.isValid()) {
                            reportError("ConstraintsGroup", "addConstraint", "The control sparsity is provided but the dimension of the two vectors do not match.");
                            return false;
                        }

                        groupControlJacobianSparsity.merge(newConstraintControlJacobianSparsity);

                    } else {
                        controlJacobianSparsityProvided = false;
                    }
                }

                SparsityStructure newConstraintStateHessianSparsity, newConstraintControlHessianSparsity, newConstraintMixedHessianSparsity;

                if (stateHessianSparsityProvided
                    && constraint->constraintSecondPartialDerivativeWRTStateSparsity(newConstraintStateHessianSparsity)) {

                    if (!newConstraintStateHessianSparsity.isValid()) {
                        reportError("ConstraintsGroup", "addConstraint", "The state hessian sparsity is provided but the dimension of the two vectors do not match.");
                        return false;
                    }

                    groupStateHessianSparsity.merge(newConstraintStateHessianSparsity);

                } else {
                    stateHessianSparsityProvided = false;
                }

                if (controlHessianSparsityProvided
                    && constraint->constraintSecondPartialDerivativeWRTControlSparsity(newConstraintControlHessianSparsity)) {

                    if (!newConstraintControlHessianSparsity.isValid()) {
                        reportError("ConstraintsGroup", "addConstraint", "The control hessian sparsity is provided but the dimension of the two vectors do not match.");
                        return false;
                    }

                    groupControlHessianSparsity.merge(newConstraintControlHessianSparsity);

                } else {
                    controlHessianSparsityProvided = false;
                }

                if (mixedHessianSparsityProvided
                    && constraint->constraintSecondPartialDerivativeWRTStateControlSparsity(newConstraintMixedHessianSparsity)) {

                    if (!newConstraintMixedHessianSparsity.isValid()) {
                        reportError("ConstraintsGroup", "addConstraint", "The state control hessian sparsity is provided but the dimension of the two vectors do not match.");
                        return false;
                    }

                    groupMixedHessianSparsity.merge(newConstraintMixedHessianSparsity);

                } else {
                    mixedHessianSparsityProvided = false;
                }

                return true;
            }

        };

        ConstraintsGroup::ConstraintsGroup(const std::string &name, unsigned int maxConstraintSize)
            :m_pimpl(new ConstraintsGroupPimpl())
        {
            assert(m_pimpl);
            m_pimpl->name = name;
            m_pimpl->maxConstraintSize = maxConstraintSize;
        }

        ConstraintsGroup::~ConstraintsGroup()
        {
            if(m_pimpl){
                delete(m_pimpl);
                m_pimpl = nullptr;
            }
        }

        const std::string ConstraintsGroup::name() const
        {
            return m_pimpl->name;
        }

        unsigned int ConstraintsGroup::constraintsDimension() const
        {
            return m_pimpl->maxConstraintSize;
        }

        bool ConstraintsGroup::addConstraint(std::shared_ptr<Constraint> constraint, const TimeRange &timeRange)
        {
            m_pimpl->isLinearGroup = false;
            return m_pimpl->addConstraint(constraint, timeRange);
        }

        bool ConstraintsGroup::addConstraint(std::shared_ptr<LinearConstraint> linearConstraint, const TimeRange &timeRange)
        {
            return m_pimpl->addConstraint(linearConstraint, timeRange);
        }

        bool ConstraintsGroup::updateTimeRange(const std::string &name, const TimeRange &timeRange)
        {
            if (timeRange == TimeRange::AnyTime()){
                if (numberOfConstraints() != 1){
                    reportError("ConstraintsGroup", "updateTimeRange",
                                "Only one constraint is allowed in a group if the timeRange is AnyTime.");
                    return false;
                }
            } else {
                if (!timeRange.isValid()){
                    reportError("ConstraintsGroup", "updateTimeRange", "Invalid timeRange.");
                    return false;
                }
            }

            GroupOfConstraintsMap::iterator constraintIterator;
            constraintIterator = m_pimpl->group.find(name);
            if(constraintIterator == m_pimpl->group.end()){
                std::ostringstream errorMsg;
                errorMsg << "Unable to find constraint named "<<name<< std::endl;
                reportError("ConstraintsGroup", "updateTimeRange", errorMsg.str().c_str());
                return false;
            }

            constraintIterator->second->timeRange = timeRange;
            std::sort(m_pimpl->orderedIntervals.begin(), m_pimpl->orderedIntervals.end(), [](const TimedConstraint_ptr&a, const TimedConstraint_ptr&b) { return a->timeRange < b->timeRange;}); //reorder the vector


            //the jacobian should be updated
            return true;
        }

        bool ConstraintsGroup::removeConstraint(const std::string& name)
        {
            if (!(m_pimpl->group.erase(name))) {
                std::ostringstream errorMsg;
                errorMsg << "Unable to find constraint named "<<name<< std::endl;
                reportError("ConstraintsGroup", "removeConstraint", errorMsg.str().c_str());
                return false;
            }

            bool found = false;
            std::vector<TimedConstraint_ptr>::iterator it = m_pimpl->orderedIntervals.begin();
            while ((!found) && (it != m_pimpl->orderedIntervals.end())){
                if (it->get()->constraint->name() == name) {
                    m_pimpl->orderedIntervals.erase(it);
                    found = true;
                } else {
                    ++it;
                }
            }

            if (!found) {
                std::ostringstream errorMsg;
                errorMsg << "Unable to find constraint named "<<name<< "on the ordered vector. This is most likely a bug." << std::endl;
                reportError("ConstraintsGroup", "removeConstraint", errorMsg.str().c_str());
                return false;
            }

            std::sort(m_pimpl->orderedIntervals.begin(), m_pimpl->orderedIntervals.end(), [](const TimedConstraint_ptr&a, const TimedConstraint_ptr&b) { return a->timeRange < b->timeRange;}); //reorder the vector
            return true;
        }

        bool ConstraintsGroup::getTimeRange(const std::string &name, TimeRange &timeRange)
        {
            GroupOfConstraintsMap::iterator constraintIterator;
            constraintIterator = m_pimpl->group.find(name);
            if (constraintIterator == m_pimpl->group.end()) {
                std::ostringstream errorMsg;
                errorMsg << "Unable to find constraint named "<<name<< std::endl;
                reportError("ConstraintsGroup", "getTimeRange", errorMsg.str().c_str());
                return false;
            }

            timeRange = constraintIterator->second->timeRange;
            return true;
        }

        std::vector<TimeRange>& ConstraintsGroup::getTimeRanges()
        {
            if (m_pimpl->timeRanges.size() != numberOfConstraints()) {
                m_pimpl->timeRanges.resize(numberOfConstraints());
            }

            size_t i=0;
            for (auto& constraint : m_pimpl->group) {
                m_pimpl->timeRanges[i] = constraint.second->timeRange;
                ++i;
            }
            return m_pimpl->timeRanges;
        }

        bool ConstraintsGroup::isFeasibilePoint(double time, const VectorDynSize &state, const VectorDynSize &control)
        {
            if (isAnyTimeGroup()) {
                return m_pimpl->group.begin()->second.get()->constraint->isFeasiblePoint(time, state, control);
            }

            std::vector< TimedConstraint_ptr >::reverse_iterator constraintIterator = m_pimpl->findActiveConstraint(time);
            if (constraintIterator == m_pimpl->orderedIntervals.rend()) {//it means that there are no constraints at that time
                return true;
            }

            return constraintIterator->get()->constraint->isFeasiblePoint(time, state, control);
        }

        bool ConstraintsGroup::evaluateConstraints(double time, const VectorDynSize &state, const VectorDynSize &control, VectorDynSize &constraints)
        {
            if (isAnyTimeGroup()) {
                if (!(m_pimpl->group.begin()->second.get()->constraint->evaluateConstraint(time, state, control, constraints))) {
                    return false;
                }
                if (constraints.size() > m_pimpl->maxConstraintSize) {
                    std::ostringstream errorMsg;
                    errorMsg << "Constraint named "<<m_pimpl->group.begin()->first<< "output a vector bigger than the specified size.";
                    reportError("ConstraintsGroup", "evaluateConstraints", errorMsg.str().c_str());
                    return false;
                }
                return true;
            }

            if (constraints.size() < m_pimpl->maxConstraintSize) {
                constraints.resize(m_pimpl->maxConstraintSize);
            }

            std::vector< TimedConstraint_ptr >::reverse_iterator constraintIterator = m_pimpl->findActiveConstraint(time);
            if (constraintIterator == m_pimpl->orderedIntervals.rend()) { //it means that there are no constraints at that time, what should be the constraint value?
                constraints.zero();
                return true;
            }
            
            if(!(constraintIterator->get()->constraint->evaluateConstraint(time, state, control, constraintIterator->get()->constraintBuffer))) {
                return false;
            }

            if (constraintIterator->get()->constraintBuffer.size() > m_pimpl->maxConstraintSize) {
                std::ostringstream errorMsg;
                errorMsg << "Constraint named "<<m_pimpl->group.begin()->first<< "output a vector bigger than the specified size.";
                reportError("ConstraintsGroup", "evaluateConstraints", errorMsg.str().c_str());
                return false;
            }

            if (constraintIterator->get()->constraint->constraintSize() < m_pimpl->maxConstraintSize) {
                toEigen(constraints).segment(0,constraintIterator->get()->constraintBuffer.size()) = toEigen(constraintIterator->get()->constraintBuffer);
                toEigen(constraints).tail(m_pimpl->maxConstraintSize - constraintIterator->get()->constraintBuffer.size()).setZero(); //append 0 at the end to equate the maxConstraintSize.
            } else {
                constraints = constraintIterator->get()->constraintBuffer;
            }

            return true;
        }

        bool ConstraintsGroup::getLowerBound(double time, VectorDynSize &lowerBound)
        {
            if (isAnyTimeGroup()) {
                return m_pimpl->group.begin()->second.get()->constraint->getLowerBound(lowerBound);
            }

            lowerBound.reserve(m_pimpl->maxConstraintSize);

            std::vector< TimedConstraint_ptr >::reverse_iterator constraintIterator = m_pimpl->findActiveConstraint(time);
            if (constraintIterator == m_pimpl->orderedIntervals.rend()) { //no active constraint
                return false; //to be considered as unbounded
            }

            if (!(constraintIterator->get()->constraint->getLowerBound(lowerBound))) {
                return false;
            }

            if (constraintIterator->get()->constraint->constraintSize() < m_pimpl->maxConstraintSize) {
                lowerBound.resize(m_pimpl->maxConstraintSize);
                for (size_t i = constraintIterator->get()->constraint->constraintSize(); i < m_pimpl->maxConstraintSize; ++i)
                    lowerBound(static_cast<unsigned int>(i)) = -1.0; //append -1.0 at the end to equate the maxConstraintSize.
            }

            return true;
        }

        bool ConstraintsGroup::getUpperBound(double time, VectorDynSize &upperBound)
        {
            if (isAnyTimeGroup()) {
                return m_pimpl->group.begin()->second.get()->constraint->getUpperBound(upperBound);
            }

            upperBound.reserve(m_pimpl->maxConstraintSize);

            std::vector< TimedConstraint_ptr >::reverse_iterator constraintIterator = m_pimpl->findActiveConstraint(time);
            if (constraintIterator == m_pimpl->orderedIntervals.rend()){ //no active constraint
                return false; //to be considered as unbounded
            }

            if (!(constraintIterator->get()->constraint->getUpperBound(upperBound))) {
                return false;
            }

            if (constraintIterator->get()->constraint->constraintSize() < m_pimpl->maxConstraintSize){
                upperBound.resize(m_pimpl->maxConstraintSize);
                for (size_t i = constraintIterator->get()->constraint->constraintSize(); i < m_pimpl->maxConstraintSize; ++i)
                    upperBound(static_cast<unsigned int>(i)) = 1.0; //append -1.0 at the end to equate the maxConstraintSize.
            }

            return true;
        }

        bool ConstraintsGroup::constraintJacobianWRTState(double time, const VectorDynSize &state, const VectorDynSize &control, MatrixDynSize &jacobian)
        {
            if (isAnyTimeGroup()) {
                TimedConstraint_ptr loneConstraint = m_pimpl->group.begin()->second;
                if (!(loneConstraint->constraint->constraintJacobianWRTState(time, state, control, jacobian))) {
                    std::ostringstream errorMsg;
                    errorMsg << "Failed to evaluate "<< loneConstraint->constraint->name() << std::endl;
                    reportError("ConstraintsGroup", "constraintJacobianWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (jacobian.rows() != loneConstraint->constraint->constraintSize()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state jacobian of constraint "<< loneConstraint->constraint->name() << " has a number of rows different from the size of the constraint." << std::endl;
                    reportError("ConstraintsGroup", "constraintJacobianWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (jacobian.cols() != state.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The state jacobian of constraint "<< loneConstraint->constraint->name() << " has a number of columns different from the state size." << std::endl;
                    reportError("ConstraintsGroup", "constraintJacobianWRTState", errorMsg.str().c_str());
                    return false;
                }

                return true;
            }

            if ((jacobian.rows() != m_pimpl->maxConstraintSize)||(jacobian.cols() != state.size())) {
                jacobian.resize(m_pimpl->maxConstraintSize, state.size());
            }

            std::vector< TimedConstraint_ptr >::reverse_iterator constraintIterator = m_pimpl->findActiveConstraint(time);
            if (constraintIterator == m_pimpl->orderedIntervals.rend()){ //no active constraint
                toEigen(jacobian).setZero();
                return true;
            }

            if (!(constraintIterator->get()->constraint->constraintJacobianWRTState(time, state, control,
                                                                                   constraintIterator->get()->stateJacobianBuffer))) {
                std::ostringstream errorMsg;
                errorMsg << "Failed to evaluate "<< constraintIterator->get()->constraint->name() << std::endl;
                reportError("ConstraintsGroup", "constraintJacobianWRTState", errorMsg.str().c_str());
                return false;
            }

            if (constraintIterator->get()->stateJacobianBuffer.rows() != constraintIterator->get()->constraint->constraintSize()) {
                std::ostringstream errorMsg;
                errorMsg << "The state jacobian of constraint "<< constraintIterator->get()->constraint->name() << " has a number of rows different from the size of the constraint." << std::endl;
                reportError("ConstraintsGroup", "constraintJacobianWRTState", errorMsg.str().c_str());
                return false;
            }

            if (constraintIterator->get()->stateJacobianBuffer.cols() != state.size()) {
                std::ostringstream errorMsg;
                errorMsg << "The state jacobian of constraint "<< constraintIterator->get()->constraint->name() << " has a number of columns different from the state size." << std::endl;
                reportError("ConstraintsGroup", "constraintJacobianWRTState", errorMsg.str().c_str());
                return false;
            }

            if (constraintIterator->get()->constraint->constraintSize() < m_pimpl->maxConstraintSize) {
                toEigen(jacobian).block(0, 0, constraintIterator->get()->stateJacobianBuffer.rows(), state.size()) =
                        toEigen(constraintIterator->get()->stateJacobianBuffer);
                unsigned int nMissing = m_pimpl->maxConstraintSize - static_cast<unsigned int>(constraintIterator->get()->constraint->constraintSize());
                toEigen(jacobian).block(constraintIterator->get()->stateJacobianBuffer.rows(), 0, nMissing, state.size()).setZero();
            } else {
                jacobian = constraintIterator->get()->stateJacobianBuffer;
            }

            return true;
        }

        bool ConstraintsGroup::constraintJacobianWRTControl(double time,
                                                            const VectorDynSize &state,
                                                            const VectorDynSize &control,
                                                            MatrixDynSize &jacobian)
        {
            if (isAnyTimeGroup()){
                TimedConstraint_ptr loneConstraint = m_pimpl->group.begin()->second;
                if (!(loneConstraint->constraint->constraintJacobianWRTControl(time, state, control, jacobian))) {
                    std::ostringstream errorMsg;
                    errorMsg << "Failed to evaluate "<< loneConstraint->constraint->name() << std::endl;
                    reportError("ConstraintsGroup", "constraintJacobianWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (jacobian.rows() != loneConstraint->constraint->constraintSize()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control jacobian of constraint "<< loneConstraint->constraint->name() << " has a number of rows different from the size of the constraint." << std::endl;
                    reportError("ConstraintsGroup", "constraintJacobianWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (jacobian.cols() != control.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The control jacobian of constraint "<< loneConstraint->constraint->name() << " has a number of columns different from the control size." << std::endl;
                    reportError("ConstraintsGroup", "constraintJacobianWRTControl", errorMsg.str().c_str());
                    return false;
                }

                return true;
            }

            if ((jacobian.rows() != m_pimpl->maxConstraintSize)||(jacobian.cols() != control.size())) {
                jacobian.resize(m_pimpl->maxConstraintSize, control.size());
            }

            std::vector< TimedConstraint_ptr >::reverse_iterator constraintIterator = m_pimpl->findActiveConstraint(time);
            if (constraintIterator == m_pimpl->orderedIntervals.rend()){ //no active constraint
                toEigen(jacobian).setZero();
                return true;
            }

            if (!(constraintIterator->get()->constraint->constraintJacobianWRTControl(time, state, control,
                                                                                     constraintIterator->get()->controlJacobianBuffer))) {
                std::ostringstream errorMsg;
                errorMsg << "Failed to evaluate "<< constraintIterator->get()->constraint->name() << std::endl;
                reportError("ConstraintsGroup", "constraintJacobianWRTControl", errorMsg.str().c_str());
                return false;
            }

            if (constraintIterator->get()->controlJacobianBuffer.rows() != constraintIterator->get()->constraint->constraintSize()) {
                std::ostringstream errorMsg;
                errorMsg << "The control jacobian of constraint "<< constraintIterator->get()->constraint->name() << " has a number of rows different from the size of the constraint." << std::endl;
                reportError("ConstraintsGroup", "constraintJacobianWRTControl", errorMsg.str().c_str());
                return false;
            }

            if (constraintIterator->get()->controlJacobianBuffer.cols() != control.size()) {
                std::ostringstream errorMsg;
                errorMsg << "The control jacobian of constraint "<< constraintIterator->get()->constraint->name() << " has a number of columns different from the control size." << std::endl;
                reportError("ConstraintsGroup", "constraintJacobianWRTControl", errorMsg.str().c_str());
                return false;
            }

            if (constraintIterator->get()->constraint->constraintSize() < m_pimpl->maxConstraintSize) {
                toEigen(jacobian).block(0, 0, constraintIterator->get()->controlJacobianBuffer.rows(), state.size()) =
                        toEigen(constraintIterator->get()->controlJacobianBuffer);
                unsigned int nMissing = m_pimpl->maxConstraintSize - static_cast<unsigned int>(constraintIterator->get()->constraint->constraintSize());
                toEigen(jacobian).block(constraintIterator->get()->controlJacobianBuffer.rows(), 0, nMissing, state.size()).setZero();
            } else {
                jacobian = constraintIterator->get()->controlJacobianBuffer;
            }

            return true;
        }

        bool ConstraintsGroup::constraintJacobianWRTStateSparsity(SparsityStructure &stateSparsity) const
        {
            if (!(m_pimpl->stateJacobianSparsityProvided)) {
                return false;
            }

            stateSparsity = m_pimpl->groupStateJacobianSparsity;

            return true;
        }

        bool ConstraintsGroup::constraintJacobianWRTControlSparsity(SparsityStructure &controlSparsity) const
        {
            if (!(m_pimpl->controlJacobianSparsityProvided)) {
                return false;
            }

            controlSparsity = m_pimpl->groupControlJacobianSparsity;

            return true;
        }

        bool ConstraintsGroup::constraintSecondPartialDerivativeWRTState(double time, const VectorDynSize &state, const VectorDynSize &control, const VectorDynSize &lambda, MatrixDynSize &hessian)
        {
            if (isAnyTimeGroup()){
                TimedConstraint_ptr loneConstraint = m_pimpl->group.begin()->second;
                if (!(loneConstraint->constraint->constraintSecondPartialDerivativeWRTState(time, state, control, lambda, hessian))) {
                    std::ostringstream errorMsg;
                    errorMsg << "Failed to evaluate "<< loneConstraint->constraint->name() << std::endl;
                    reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != state.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The second partial derivative WRT the state of constraint "<< loneConstraint->constraint->name() << " has a number of rows different from the size of the state." << std::endl;
                    reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.cols() != state.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The second partial derivative WRT the state of constraint "<< loneConstraint->constraint->name() << " has a number of columns different from the size of the state." << std::endl;
                    reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                    return false;
                }

                return true;
            }

            hessian.resize(state.size(), state.size());

            std::vector< TimedConstraint_ptr >::reverse_iterator constraintIterator = m_pimpl->findActiveConstraint(time);
            if (constraintIterator == m_pimpl->orderedIntervals.rend()){ //no active constraint
                toEigen(hessian).setZero();
                return true;
            }

            toEigen(constraintIterator->get()->lambdaBuffer) = toEigen(lambda).topRows(constraintIterator->get()->lambdaBuffer.size());

            if (!(constraintIterator->get()->constraint->constraintSecondPartialDerivativeWRTState(time, state, control, constraintIterator->get()->lambdaBuffer, hessian))) {
                std::ostringstream errorMsg;
                errorMsg << "Failed to evaluate "<< constraintIterator->get()->constraint->name() << std::endl;
                reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                return false;
            }

            if (hessian.rows() != state.size()) {
                std::ostringstream errorMsg;
                errorMsg << "The second partial derivative WRT the state of constraint "<< constraintIterator->get()->constraint->name() << " has a number of rows different from the size of the state." << std::endl;
                reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                return false;
            }

            if (hessian.cols() != state.size()) {
                std::ostringstream errorMsg;
                errorMsg << "The second partial derivative WRT the state of constraint "<< constraintIterator->get()->constraint->name() << " has a number of columns different from the size of the state." << std::endl;
                reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                return false;
            }

            return true;
        }

        bool ConstraintsGroup::constraintSecondPartialDerivativeWRTControl(double time, const VectorDynSize &state, const VectorDynSize &control, const VectorDynSize &lambda, MatrixDynSize &hessian)
        {
            if (isAnyTimeGroup()){
                TimedConstraint_ptr loneConstraint = m_pimpl->group.begin()->second;
                if (!(loneConstraint->constraint->constraintSecondPartialDerivativeWRTControl(time, state, control, lambda, hessian))) {
                    std::ostringstream errorMsg;
                    errorMsg << "Failed to evaluate "<< loneConstraint->constraint->name() << std::endl;
                    reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != control.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The second partial derivative WRT the control of constraint "<< loneConstraint->constraint->name() << " has a number of rows different from the size of the control." << std::endl;
                    reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.cols() != control.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The second partial derivative WRT the control of constraint "<< loneConstraint->constraint->name() << " has a number of columns different from the size of the control." << std::endl;
                    reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                    return false;
                }

                return true;
            }

            hessian.resize(control.size(), control.size());

            std::vector< TimedConstraint_ptr >::reverse_iterator constraintIterator = m_pimpl->findActiveConstraint(time);
            if (constraintIterator == m_pimpl->orderedIntervals.rend()){ //no active constraint
                toEigen(hessian).setZero();
                return true;
            }

            toEigen(constraintIterator->get()->lambdaBuffer) = toEigen(lambda).topRows(constraintIterator->get()->lambdaBuffer.size());

            if (!(constraintIterator->get()->constraint->constraintSecondPartialDerivativeWRTControl(time, state, control, constraintIterator->get()->lambdaBuffer, hessian))) {
                std::ostringstream errorMsg;
                errorMsg << "Failed to evaluate "<< constraintIterator->get()->constraint->name() << std::endl;
                reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                return false;
            }

            if (hessian.rows() != control.size()) {
                std::ostringstream errorMsg;
                errorMsg << "The second partial derivative WRT the control of constraint "<< constraintIterator->get()->constraint->name() << " has a number of rows different from the size of the control." << std::endl;
                reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                return false;
            }

            if (hessian.cols() != control.size()) {
                std::ostringstream errorMsg;
                errorMsg << "The second partial derivative WRT the control of constraint "<< constraintIterator->get()->constraint->name() << " has a number of columns different from the size of the control." << std::endl;
                reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                return false;
            }

            return true;
        }

        bool ConstraintsGroup::constraintSecondPartialDerivativeWRTStateControl(double time, const VectorDynSize &state, const VectorDynSize &control, const VectorDynSize &lambda, MatrixDynSize &hessian)
        {
            if (isAnyTimeGroup()){
                TimedConstraint_ptr loneConstraint = m_pimpl->group.begin()->second;
                if (!(loneConstraint->constraint->constraintSecondPartialDerivativeWRTStateControl(time, state, control, lambda, hessian))) {
                    std::ostringstream errorMsg;
                    errorMsg << "Failed to evaluate "<< loneConstraint->constraint->name() << std::endl;
                    reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTStateControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.rows() != state.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The second partial derivative WRT state/control of constraint "<< loneConstraint->constraint->name() << " has a number of rows different from the size of the state." << std::endl;
                    reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTStateControl", errorMsg.str().c_str());
                    return false;
                }

                if (hessian.cols() != control.size()) {
                    std::ostringstream errorMsg;
                    errorMsg << "The second partial derivative WRT state/control of constraint "<< loneConstraint->constraint->name() << " has a number of columns different from the size of the control." << std::endl;
                    reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTStateControl", errorMsg.str().c_str());
                    return false;
                }

                return true;
            }

            hessian.resize(state.size(), control.size());

            std::vector< TimedConstraint_ptr >::reverse_iterator constraintIterator = m_pimpl->findActiveConstraint(time);
            if (constraintIterator == m_pimpl->orderedIntervals.rend()){ //no active constraint
                toEigen(hessian).setZero();
                return true;
            }

            toEigen(constraintIterator->get()->lambdaBuffer) = toEigen(lambda).topRows(constraintIterator->get()->lambdaBuffer.size());

            if (!(constraintIterator->get()->constraint->constraintSecondPartialDerivativeWRTStateControl(time, state, control, constraintIterator->get()->lambdaBuffer, hessian))) {
                std::ostringstream errorMsg;
                errorMsg << "Failed to evaluate "<< constraintIterator->get()->constraint->name() << std::endl;
                reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTStateControl", errorMsg.str().c_str());
                return false;
            }

            if (hessian.rows() != state.size()) {
                std::ostringstream errorMsg;
                errorMsg << "The second partial derivative WRT state/control of constraint "<< constraintIterator->get()->constraint->name() << " has a number of rows different from the size of the state." << std::endl;
                reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTStateControl", errorMsg.str().c_str());
                return false;
            }

            if (hessian.cols() != control.size()) {
                std::ostringstream errorMsg;
                errorMsg << "The second partial derivative WRT state/control of constraint "<< constraintIterator->get()->constraint->name() << " has a number of columns different from the size of the control." << std::endl;
                reportError("ConstraintsGroup", "constraintSecondPartialDerivativeWRTStateControl", errorMsg.str().c_str());
                return false;
            }

            return true;
        }

        bool ConstraintsGroup::constraintsSecondPartialDerivativeWRTStateSparsity(SparsityStructure &stateSparsity)
        {
            if (!(m_pimpl->stateHessianSparsityProvided)) {
                return false;
            }

            stateSparsity = m_pimpl->groupStateHessianSparsity;

            return true;
        }

        bool ConstraintsGroup::constraintsSecondPartialDerivativeWRTStateControlSparsity(SparsityStructure &stateControlSparsity)
        {
            if (!(m_pimpl->mixedHessianSparsityProvided)) {
                return false;
            }

            stateControlSparsity = m_pimpl->groupMixedHessianSparsity;

            return true;
        }

        bool ConstraintsGroup::constraintsSecondPartialDerivativeWRTControlSparsity(SparsityStructure &controlSparsity)
        {
            if (!(m_pimpl->controlHessianSparsityProvided)) {
                return false;
            }

            controlSparsity = m_pimpl->groupControlHessianSparsity;

            return true;
        }

        bool ConstraintsGroup::isAnyTimeGroup()
        {
            if ((numberOfConstraints() == 1) && (m_pimpl->group.begin()->second.get()->timeRange == TimeRange::AnyTime())) {
                return true;
            }
            return false;
        }

        unsigned int ConstraintsGroup::numberOfConstraints() const
        {
            return static_cast<unsigned int>(m_pimpl->group.size());
        }

        const std::vector<std::string> ConstraintsGroup::listConstraints() const
        {
            std::vector<std::string> output;
            for (auto& constraint: m_pimpl->group) {
                output.push_back(constraint.second->constraint->name()); //MEMORY ALLOCATION
            }
            return output;
        }

        bool ConstraintsGroup::isLinearGroup() const
        {
            return m_pimpl->isLinearGroup;
        }

    }
}
