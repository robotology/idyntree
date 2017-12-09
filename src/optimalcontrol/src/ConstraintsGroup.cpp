/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include "iDynTree/Core/VectorDynSize.h"
#include "iDynTree/Core/Utils.h"
#include "iDynTree/ConstraintsGroup.h"
#include "iDynTree/Constraint.h"
#include "iDynTree/TimeRange.h"
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
        }TimedConstraint;

        typedef std::shared_ptr< TimedConstraint> TimedConstraint_ptr;

        typedef std::unordered_map< std::string, TimedConstraint_ptr> GroupOfConstraintsMap;

        class ConstraintsGroup::ConstraintsGroupPimpl{
        public:
            GroupOfConstraintsMap group;
            std::vector< TimedConstraint_ptr > orderedIntervals;
            std::string name;
            unsigned int maxConstraintSize;
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

        bool ConstraintsGroup::addConstraint(std::shared_ptr<Constraint> constraint, const TimeRange &timeRange)
        {
            if (constraint->constraintSize() > m_pimpl->maxConstraintSize){
                reportError("ConstraintsGroup", "addConstraint", "The constraint dimension is greater than the maximum allowed by the group.");
                return false;
            }

            if (timeRange == TimeRange::AnyTime()){
                if (numberOfConstraints() != 0){
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

            if(m_pimpl->group.size() > 0){
                GroupOfConstraintsMap::iterator constraintIterator;
                constraintIterator = m_pimpl->group.find(constraint->name());
                if(constraintIterator != m_pimpl->group.end()){
                    std::ostringstream errorMsg;
                    errorMsg << "A constraint named " << constraint->name()
                             <<" already exists in the group "<< m_pimpl->name << " .";
                    reportError("ConstraintsGroup", "addConstraint", errorMsg.str().c_str());
                    return false;
                }
            }

            //add constraints in the group
            TimedConstraint_ptr newConstraint = std::make_shared<TimedConstraint>();

            newConstraint->timeRange = timeRange;
            newConstraint->constraint = constraint;

            std::pair< GroupOfConstraintsMap::iterator, bool> result;
            result = m_pimpl->group.insert(GroupOfConstraintsMap::value_type(constraint->name(), newConstraint));

            if(!result.second){
                std::ostringstream errorMsg;
                errorMsg << "Unable to add constraint "<<constraint->name() << std::endl;
                reportError("ConstraintsGroup", "addConstraint", errorMsg.str().c_str());
                return false;
            }

            m_pimpl->orderedIntervals.push_back(result.first->second); //register the time range in order to have the constraints ordered by init time. result.first->second is the TimedConstraint_ptr of the newly inserted TimedConstraint.
            std::sort(m_pimpl->orderedIntervals.begin(), m_pimpl->orderedIntervals.end(), [](const TimedConstraint_ptr&a, const TimedConstraint_ptr&b) { return a->timeRange < b->timeRange;}); //reorder the vector


            //the jacobian should be updated

            return true;
        }

        bool ConstraintsGroup::updateTimeRange(const std::string &name, const TimeRange &timeRange)
        {
            if (timeRange == TimeRange::AnyTime()){
                if (numberOfConstraints() != 1){
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
            if(!(m_pimpl->group.erase(name))){
                std::ostringstream errorMsg;
                errorMsg << "Unable to find constraint named "<<name<< std::endl;
                reportError("ConstraintsGroup", "removeConstraint", errorMsg.str().c_str());
                return false;
            }
            return true;
        }

        bool ConstraintsGroup::getTimeRange(const std::string &name, TimeRange &timeRange)
        {
            GroupOfConstraintsMap::iterator constraintIterator;
            constraintIterator = m_pimpl->group.find(name);
            if(constraintIterator == m_pimpl->group.end()){
                std::ostringstream errorMsg;
                errorMsg << "Unable to find constraint named "<<name<< std::endl;
                reportError("ConstraintsGroup", "getTimeRange", errorMsg.str().c_str());
                return false;
            }

            timeRange = constraintIterator->second->timeRange;
            return true;
        }

        bool ConstraintsGroup::isFeasibilePoint(double time, const VectorDynSize &state, const VectorDynSize &control)
        {
            if (isAnyTimeGroup()){
                return m_pimpl->group.begin()->second.get()->constraint->isFeasiblePoint(time, state, control);
            }

            std::vector< TimedConstraint_ptr >::reverse_iterator constraintIterator =
                    std::find_if(m_pimpl->orderedIntervals.rbegin(),
                                 m_pimpl->orderedIntervals.rend(),
                                 [time](const TimedConstraint_ptr & a) -> bool { return a->timeRange.isInRange(time); }); //find the last element in the vector with init time lower than the specified time
            if (constraintIterator == m_pimpl->orderedIntervals.rend()) //it means that there are no constraints at that time
                return true;

            return constraintIterator->get()->constraint->isFeasiblePoint(time, state, control);
        }

        bool ConstraintsGroup::evaluateConstraints(double time, const VectorDynSize &state, const VectorDynSize &control, VectorDynSize &constraints)
        {
            if (isAnyTimeGroup()){
                return m_pimpl->group.begin()->second.get()->constraint->evaluateConstraint(time, state, control, constraints);
            }

            constraints.reserve(m_pimpl->maxConstraintSize);

            std::vector< TimedConstraint_ptr >::reverse_iterator constraintIterator =
                    std::find_if(m_pimpl->orderedIntervals.rbegin(),
                                 m_pimpl->orderedIntervals.rend(),
                                 [time](const TimedConstraint_ptr & a) -> bool { return a->timeRange.isInRange(time); }); //find the last element in the vector with init time lower than the specified time
            if (constraintIterator == m_pimpl->orderedIntervals.rend()){ //it means that there are no constraints at that time, what should be the constraint value?
                std::ostringstream errorMsg;
                errorMsg << "No constraint defined at time "<<time<< std::endl;
                reportError("ConstraintsGroup", "evaluateConstraints", errorMsg.str().c_str());
                return false;
            }
            
            if(!(constraintIterator->get()->constraint->evaluateConstraint(time, state, control, constraints)))
                return false;

            if (constraintIterator->get()->constraint->constraintSize() < m_pimpl->maxConstraintSize){
                constraints.resize(m_pimpl->maxConstraintSize);
                for (size_t i = constraintIterator->get()->constraint->constraintSize(); i < m_pimpl->maxConstraintSize; ++i)
                    constraints(i) = 0.0; //append 0 at the end to equate the maxConstraintSize.
            }

            return true;
        }

        bool ConstraintsGroup::isAnyTimeGroup()
        {
            if ((numberOfConstraints() == 1) && (m_pimpl->group.begin()->second.get()->timeRange == TimeRange::AnyTime()))
                return true;
            return false;
        }

        unsigned int ConstraintsGroup::numberOfConstraints() const
        {
            return m_pimpl->group.size();
        }

        const std::vector<std::string> ConstraintsGroup::listConstraints() const
        {
            std::vector<std::string> output;
            for(auto constraint: m_pimpl->group)
                output.push_back(constraint.second->constraint->name()); //MEMORY ALLOCATION
            return output;
        }

    }
}
