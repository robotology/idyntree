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

#include <iDynTree/OptimalControlProblem.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/LinearSystem.h>
#include <iDynTree/ConstraintsGroup.h>
#include <iDynTree/Constraint.h>
#include <iDynTree/LinearConstraint.h>
#include <iDynTree/Cost.h>
#include <iDynTree/QuadraticLikeCost.h>
#include <iDynTree/QuadraticCost.h>
#include <iDynTree/L2NormCost.h>
#include <iDynTree/LinearCost.h>
#include <iDynTree/TimeRange.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Utils.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <Eigen/Dense>

#include <map>
#include <cassert>
#include <sstream>


namespace iDynTree {
    namespace optimalcontrol {

        typedef struct {
            std::shared_ptr<ConstraintsGroup> group_ptr;
            VectorDynSize constraintsBuffer;
            MatrixDynSize stateJacobianBuffer, controlJacobianBuffer;
            SparsityStructure stateSparsity;
            SparsityStructure controlSparsity;
            bool hasStateSparsity = false;
            bool hasControlSparsity = false;
        } BufferedGroup;

        typedef std::shared_ptr<BufferedGroup> BufferedGroup_ptr;

        typedef std::map< std::string, BufferedGroup_ptr> ConstraintsGroupsMap;

        typedef struct{
            std::shared_ptr<Cost> cost;
            double weight;
            TimeRange timeRange;
            bool isLinear;
            bool isQuadratic;
        } TimedCost;

        typedef std::map< std::string, TimedCost> CostsMap;

        class OptimalControlProblem::OptimalControlProblemPimpl
        {
        public:
            TimeRange horizon;
            std::shared_ptr<DynamicalSystem> dynamicalSystem;
            ConstraintsGroupsMap constraintsGroups;
            CostsMap costs;
            VectorDynSize costStateJacobianBuffer, costControlJacobianBuffer;
            MatrixDynSize costStateHessianBuffer, costControlHessianBuffer, costMixedHessianBuffer;
            bool stateLowerBounded, stateUpperBounded, controlLowerBounded, controlUpperBounded;
            std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingVector> stateLowerBound, stateUpperBound, controlLowerBound, controlUpperBound; //if they are empty is like there is no bound
            SparsityStructure stateSparsity;
            SparsityStructure controlSparsity;
            std::vector<std::string> mayerCostnames;
            std::vector<TimeRange> constraintsTimeRanges, costTimeRanges;
            std::vector<size_t> linearConstraintIndeces;
            bool systemIsLinear = false;

            bool addCost(double weight, const TimeRange& timeRange, std::shared_ptr<Cost> cost, bool isLinear, bool isQuadratic, const std::string& methodName) {
                if (!cost){
                    reportError("OptimalControlProblem", methodName.c_str(), "Empty cost pointer");
                    return false;
                }

                TimedCost newCost;
                newCost.cost = cost;
                newCost.weight = weight;
                newCost.timeRange = timeRange;
                newCost.isLinear = isLinear;
                newCost.isQuadratic = isQuadratic;

                std::pair<CostsMap::iterator, bool> costResult;
                costResult = costs.insert(std::pair<std::string, TimedCost>(cost->name(), newCost));
                if(!costResult.second){
                    std::ostringstream errorMsg;
                    errorMsg << "A cost named " << cost->name() <<" already exists.";
                    reportError("OptimalControlProblem", methodName.c_str(), errorMsg.str().c_str());
                    return false;
                }
                return true;
            }

        };


        //MARK: OptimalControlProblem implementation

        OptimalControlProblem::OptimalControlProblem()
            : m_pimpl(new OptimalControlProblemPimpl())
        {
            assert(m_pimpl);
            m_pimpl->horizon = TimeRange::Instant(0.0);
            m_pimpl->dynamicalSystem = nullptr;
            m_pimpl->stateLowerBounded = false;
            m_pimpl->stateUpperBounded = false;
            m_pimpl->controlLowerBounded = false;
            m_pimpl->controlUpperBounded = false;

        }

        OptimalControlProblem::~OptimalControlProblem()
        {
            if(m_pimpl){
                delete(m_pimpl);
                m_pimpl = nullptr;
            }
        }

        bool OptimalControlProblem::setTimeHorizon(double startingTime, double finalTime)
        {
            if (!(m_pimpl->horizon.setTimeInterval(startingTime,finalTime))) {
                return false;
            }

            CostsMap::iterator costIterator;
            for (auto& mayerCost : m_pimpl->mayerCostnames){
                if (mayerCost.size() > 0){
                    costIterator = m_pimpl->costs.find(mayerCost);
                    if (costIterator == m_pimpl->costs.end()){
                        std::ostringstream errorMsg;
                        errorMsg << "Unable to find cost named "<<mayerCost<< std::endl;
                        reportError("OptimalControlProblem", "updateCostTimeRange", errorMsg.str().c_str());
                        return false;
                    }

                    costIterator->second.timeRange = TimeRange::Instant(finalTime);
                }
            }

            return true;
        }

        double OptimalControlProblem::initialTime() const
        {
            return m_pimpl->horizon.initTime();
        }

        double OptimalControlProblem::finalTime() const
        {
            return m_pimpl->horizon.endTime();
        }

        bool OptimalControlProblem::setDynamicalSystemConstraint(std::shared_ptr<DynamicalSystem> dynamicalSystem)
        {
            if (m_pimpl->dynamicalSystem){
                reportError("OptimalControlProblem", "setDynamicalSystemConstraint", "Change dynamical system is forbidden.");
                return false;
            }
            m_pimpl->dynamicalSystem = dynamicalSystem;
            return true;
        }

        bool OptimalControlProblem::setDynamicalSystemConstraint(std::shared_ptr<LinearSystem> linearSystem)
        {
            if (m_pimpl->dynamicalSystem){
                reportError("OptimalControlProblem", "setDynamicalSystemConstraint", "Change dynamical system is forbidden.");
                return false;
            }
            m_pimpl->dynamicalSystem = linearSystem;
            m_pimpl->systemIsLinear = true;
            return true;
        }

        const std::weak_ptr<DynamicalSystem> OptimalControlProblem::dynamicalSystem() const
        {
            return m_pimpl->dynamicalSystem;
        }

        bool OptimalControlProblem::systemIsLinear() const
        {
            return m_pimpl->systemIsLinear;
        }

        bool OptimalControlProblem::addGroupOfConstraints(std::shared_ptr<ConstraintsGroup> groupOfConstraints)
        {
            if (!groupOfConstraints){
                 reportError("OptimalControlProblem", "addGroupOfConstraints", "Empty group pointer");
                 return false;
            }

            BufferedGroup_ptr newGroup = std::make_shared<BufferedGroup>();
            newGroup->group_ptr = groupOfConstraints;
            newGroup->constraintsBuffer.resize(groupOfConstraints->constraintsDimension());
            if (m_pimpl->dynamicalSystem) {
                newGroup->stateJacobianBuffer.resize(groupOfConstraints->constraintsDimension(), static_cast<unsigned int>(m_pimpl->dynamicalSystem->stateSpaceSize()));
                newGroup->controlJacobianBuffer.resize(groupOfConstraints->constraintsDimension(), static_cast<unsigned int>(m_pimpl->dynamicalSystem->controlSpaceSize()));
            }

            newGroup->hasStateSparsity = groupOfConstraints->constraintJacobianWRTStateSparsity(newGroup->stateSparsity); //needed to allocate memory in advance
            newGroup->hasControlSparsity = groupOfConstraints->constraintJacobianWRTControlSparsity(newGroup->controlSparsity); //needed to allocate memory in advance

            std::pair< ConstraintsGroupsMap::iterator, bool> groupResult;
            groupResult = m_pimpl->constraintsGroups.insert(std::pair< std::string, BufferedGroup_ptr>(groupOfConstraints->name(), newGroup));

            if(!groupResult.second){
                std::ostringstream errorMsg;
                errorMsg << "A group (or a constraint) named " << groupOfConstraints->name() <<" already exists.";
                reportError("OptimalControlProblem", "addGroupOfConstraints", errorMsg.str().c_str());
                return false;
            }

            m_pimpl->stateSparsity.resize(m_pimpl->stateSparsity.size() + newGroup->stateSparsity.size()); //needed to allocate memory in advance

            m_pimpl->controlSparsity.resize(m_pimpl->controlSparsity.size() + newGroup->controlSparsity.size()); //needed to allocate memory in advance

            return true;
        }

        bool OptimalControlProblem::removeGroupOfConstraints(const std::string &name)
        {
            ConstraintsGroupsMap::iterator groupIterator;
            groupIterator = m_pimpl->constraintsGroups.find(name);
            if(groupIterator != m_pimpl->constraintsGroups.end()){
                if(m_pimpl->constraintsGroups.erase(name)){
                    return true;
                } else {
                    std::ostringstream errorMsg;
                    errorMsg << "Failed to group named "<<name<< std::endl;
                    reportError("OptimalControlProblem", "removeGroupOfConstraints", errorMsg.str().c_str());
                    return false;
                }
            }

            std::ostringstream errorMsg;
            errorMsg << "Unable to find group named "<<name<< std::endl;
            reportError("OptimalControlProblem", "removeGroupOfConstraints", errorMsg.str().c_str());
            return false;
        }

        bool OptimalControlProblem::addConstraint(std::shared_ptr<Constraint> newConstraint)
        {
            if (!newConstraint){
                reportError("OptimalControlProblem", "addConstraint", "Invalid constraint pointer.");
                return false;
            }

            std::shared_ptr<ConstraintsGroup> dummyGroup =
                    std::make_shared<ConstraintsGroup>(newConstraint->name(),newConstraint->constraintSize());

            if(!dummyGroup){
                reportError("OptimalControlProblem", "addConstraint", "Failed in adding constraint.");
                return false;
            }

            if(!(dummyGroup->addConstraint(newConstraint, TimeRange::AnyTime()))){
                return false;
            }
            return addGroupOfConstraints(dummyGroup);
        }

        bool OptimalControlProblem::addConstraint(std::shared_ptr<LinearConstraint> newConstraint)
        {
            if (!newConstraint){
                reportError("OptimalControlProblem", "addConstraint", "Invalid constraint pointer.");
                return false;
            }

            std::shared_ptr<ConstraintsGroup> dummyGroup =
                    std::make_shared<ConstraintsGroup>(newConstraint->name(),newConstraint->constraintSize());

            if(!dummyGroup){
                reportError("OptimalControlProblem", "addConstraint", "Failed in adding constraint.");
                return false;
            }

            if(!(dummyGroup->addConstraint(newConstraint, TimeRange::AnyTime()))){ //here is important to pass a LinearConstraint pointer
                return false;
            }
            return addGroupOfConstraints(dummyGroup);
        }

        bool OptimalControlProblem::removeConstraint(const std::string &name)
        {
            ConstraintsGroupsMap::iterator groupIterator;
            groupIterator = m_pimpl->constraintsGroups.find(name);
            if(groupIterator != m_pimpl->constraintsGroups.end()){
                if(groupIterator->second->group_ptr->isAnyTimeGroup()){
                    if(m_pimpl->constraintsGroups.erase(name)){
                        return true;
                    }
                    else {
                        std::ostringstream errorMsg;
                        errorMsg << "Failed to remove constraint named "<<name<< std::endl;
                        reportError("OptimalControlProblem", "removeConstraint", errorMsg.str().c_str());
                        return false;
                    }
                } else {
                    std::ostringstream errorMsg;
                    errorMsg << name << " is a group. To remove it use the method removeGroupOfConstraints instead."<< std::endl;
                    reportError("OptimalControlProblem", "removeConstraint", errorMsg.str().c_str());
                    return false;
                }

            }
            std::ostringstream errorMsg;
            errorMsg << "Unable to find constraint named "<<name<< std::endl;
            reportError("OptimalControlProblem", "removeConstraint", errorMsg.str().c_str());
            return false;
        }

        unsigned int OptimalControlProblem::countConstraints() const
        {
            unsigned int number = 0;

            for(auto& group: m_pimpl->constraintsGroups){
                number += group.second->group_ptr->numberOfConstraints();
            }

            return number;
        }

        unsigned int OptimalControlProblem::countLinearConstraints() const
        {
            unsigned int number = 0;

            for(auto& group: m_pimpl->constraintsGroups){
                if (group.second->group_ptr->isLinearGroup()) {
                    number += group.second->group_ptr->constraintsDimension();
                }
            }

            return number;
        }

        unsigned int OptimalControlProblem::getConstraintsDimension() const
        {
            unsigned int dimension = 0;

            for (auto& group: m_pimpl->constraintsGroups){
                dimension += group.second->group_ptr->constraintsDimension();
            }
            return dimension;
        }

        const std::vector<std::string> OptimalControlProblem::listConstraints() const
        {
            std::vector<std::string> output;
            std::vector<std::string> temp;

            for(auto& group: m_pimpl->constraintsGroups){
                temp =  group.second->group_ptr->listConstraints();
                output.insert(output.end(), temp.begin(), temp.end());
            }
            return output;
        }

        const std::vector<std::string> OptimalControlProblem::listGroups() const
        {
            std::vector<std::string> output;

            for(auto& group: m_pimpl->constraintsGroups){
                output.insert(output.end(), group.second->group_ptr->numberOfConstraints(), group.second->group_ptr->name());
            }
            return output;
        }

        std::vector<TimeRange> &OptimalControlProblem::getConstraintsTimeRanges() const
        {
            unsigned int nc = countConstraints();

            if (m_pimpl->constraintsTimeRanges.size() != nc)
                m_pimpl->constraintsTimeRanges.resize(nc);

            size_t index = 0;

            for (auto& group : m_pimpl->constraintsGroups){
                std::vector<TimeRange> &groupTimeRanges = group.second->group_ptr->getTimeRanges();
                for (size_t i = 0; i < groupTimeRanges.size(); ++i){
                    m_pimpl->constraintsTimeRanges[index] = groupTimeRanges[i];
                    ++index;
                }
            }
            return m_pimpl->constraintsTimeRanges;
        }

        std::vector<size_t> &OptimalControlProblem::getLinearConstraintsIndeces() const
        {
            unsigned int nl = countLinearConstraints();

            if (m_pimpl->linearConstraintIndeces.size() != nl)
                m_pimpl->linearConstraintIndeces.resize(nl);

            size_t vectorIndex = 0, constraintIndex = 0;

            unsigned int nc = 0;
            for (auto& group : m_pimpl->constraintsGroups){
                nc = group.second->group_ptr->constraintsDimension();

                if (group.second->group_ptr->isLinearGroup()) {
                    for (size_t i = 0; i < nc; ++i){
                        m_pimpl->linearConstraintIndeces[vectorIndex] = constraintIndex + i;
                        ++vectorIndex;
                    }
                }
                constraintIndex += nc;
            }
            return m_pimpl->linearConstraintIndeces;
        }

        bool OptimalControlProblem::addMayerTerm(double weight, std::shared_ptr<Cost> cost)
        {
            TimeRange newTimerange(m_pimpl->horizon.endTime(), m_pimpl->horizon.endTime());
            if (!(m_pimpl->addCost(weight, newTimerange, cost, false, false, "addMayerTerm"))) {
                return false;
            }
            m_pimpl->mayerCostnames.push_back(cost->name());

            return true;
        }

        bool OptimalControlProblem::addMayerTerm(double weight, std::shared_ptr<QuadraticCost> quadraticCost)
        {
            TimeRange newTimerange(m_pimpl->horizon.endTime(), m_pimpl->horizon.endTime());
            if (!(m_pimpl->addCost(weight, newTimerange, quadraticCost, false, true, "addMayerTerm"))) {
                return false;
            }
            m_pimpl->mayerCostnames.push_back(quadraticCost->name());

            return true;
        }

        bool OptimalControlProblem::addMayerTerm(double weight, std::shared_ptr<L2NormCost> quadraticCost)
        {
            TimeRange newTimerange(m_pimpl->horizon.endTime(), m_pimpl->horizon.endTime());
            if (!(m_pimpl->addCost(weight, newTimerange, quadraticCost, false, true, "addMayerTerm"))) {
                return false;
            }
            m_pimpl->mayerCostnames.push_back(quadraticCost->name());

            return true;
        }

        bool OptimalControlProblem::addMayerTerm(double weight, std::shared_ptr<LinearCost> linearCost)
        {
            TimeRange newTimerange(m_pimpl->horizon.endTime(), m_pimpl->horizon.endTime());
            if (!(m_pimpl->addCost(weight, newTimerange, linearCost, true, true, "addMayerTerm"))) {
                return false;
            }
            m_pimpl->mayerCostnames.push_back(linearCost->name());

            return true;
        }

        bool OptimalControlProblem::addLagrangeTerm(double weight, std::shared_ptr<Cost> cost)
        {
            return (m_pimpl->addCost(weight, TimeRange::AnyTime(), cost, false, false, "addLagrangeTerm"));
        }

        bool OptimalControlProblem::addLagrangeTerm(double weight, std::shared_ptr<QuadraticCost> quadraticCost)
        {
            return (m_pimpl->addCost(weight, TimeRange::AnyTime(), quadraticCost, false, true, "addLagrangeTerm"));
        }

        bool OptimalControlProblem::addLagrangeTerm(double weight, std::shared_ptr<L2NormCost> quadraticCost)
        {
            return (m_pimpl->addCost(weight, TimeRange::AnyTime(), quadraticCost, false, true, "addLagrangeTerm"));
        }

        bool OptimalControlProblem::addLagrangeTerm(double weight, std::shared_ptr<LinearCost> linearCost)
        {
            return (m_pimpl->addCost(weight, TimeRange::AnyTime(), linearCost, true, true, "addLagrangeTerm"));
        }

        bool OptimalControlProblem::addLagrangeTerm(double weight, double startingTime, double finalTime, std::shared_ptr<Cost> cost)
        {
            TimeRange timeRange;

            if (!timeRange.setTimeInterval(startingTime, finalTime)){
                std::ostringstream errorMsg;
                errorMsg << "The cost named " << cost->name() <<" has invalid time settings.";
                reportError("OptimalControlProblem", "addLagrangeTerm", errorMsg.str().c_str());
                return false;
            }

            return (m_pimpl->addCost(weight, timeRange, cost, false, false, "addLagrangeTerm"));
        }

        bool OptimalControlProblem::addLagrangeTerm(double weight, double startingTime, double finalTime, std::shared_ptr<QuadraticCost> quadraticCost)
        {
            TimeRange timeRange;

            if (!timeRange.setTimeInterval(startingTime, finalTime)){
                std::ostringstream errorMsg;
                errorMsg << "The cost named " << quadraticCost->name() <<" has invalid time settings.";
                reportError("OptimalControlProblem", "addLagrangeTerm", errorMsg.str().c_str());
                return false;
            }

            return (m_pimpl->addCost(weight, timeRange, quadraticCost, false, true, "addLagrangeTerm"));
        }

        bool OptimalControlProblem::addLagrangeTerm(double weight, double startingTime, double finalTime, std::shared_ptr<L2NormCost> quadraticCost)
        {
            TimeRange timeRange;

            if (!timeRange.setTimeInterval(startingTime, finalTime)){
                std::ostringstream errorMsg;
                errorMsg << "The cost named " << quadraticCost->name() <<" has invalid time settings.";
                reportError("OptimalControlProblem", "addLagrangeTerm", errorMsg.str().c_str());
                return false;
            }

            return (m_pimpl->addCost(weight, timeRange, quadraticCost, false, true, "addLagrangeTerm"));
        }

        bool OptimalControlProblem::addLagrangeTerm(double weight, double startingTime, double finalTime, std::shared_ptr<LinearCost> linearCost)
        {
            TimeRange timeRange;

            if (!timeRange.setTimeInterval(startingTime, finalTime)){
                std::ostringstream errorMsg;
                errorMsg << "The cost named " << linearCost->name() <<" has invalid time settings.";
                reportError("OptimalControlProblem", "addLagrangeTerm", errorMsg.str().c_str());
                return false;
            }

            return (m_pimpl->addCost(weight, timeRange, linearCost, true, true, "addLagrangeTerm"));
        }

        bool OptimalControlProblem::addLagrangeTerm(double weight, const TimeRange &timeRange, std::shared_ptr<Cost> cost)
        {
            return (m_pimpl->addCost(weight, timeRange, cost, false, false, "addLagrangeTerm"));
        }

        bool OptimalControlProblem::addLagrangeTerm(double weight, const TimeRange &timeRange, std::shared_ptr<QuadraticCost> quadraticCost)
        {
            return (m_pimpl->addCost(weight, timeRange, quadraticCost, false, true, "addLagrangeTerm"));
        }

        bool OptimalControlProblem::addLagrangeTerm(double weight, const TimeRange &timeRange, std::shared_ptr<L2NormCost> quadraticCost)
        {
            return (m_pimpl->addCost(weight, timeRange, quadraticCost, false, true, "addLagrangeTerm"));
        }

        bool OptimalControlProblem::addLagrangeTerm(double weight, const TimeRange &timeRange, std::shared_ptr<LinearCost> linearCost)
        {
            return (m_pimpl->addCost(weight, timeRange, linearCost, true, true, "addLagrangeTerm"));
        }

        bool OptimalControlProblem::updateCostTimeRange(const std::string &name, double newStartingTime, double newEndTime)
        {
            TimeRange timeRange;

            if (!timeRange.setTimeInterval(newStartingTime, newEndTime)){
                std::ostringstream errorMsg;
                errorMsg << "Invalid time settings. Cannot apply changes to cost " << name <<".";
                reportError("OptimalControlProblem", "updateCostTimeRange", errorMsg.str().c_str());
                return false;
            }

            return updateCostTimeRange(name, timeRange);
        }

        bool OptimalControlProblem::updateCostTimeRange(const std::string &name, const TimeRange &newTimeRange)
        {
            for (auto& mayerCost : m_pimpl->mayerCostnames)
                if (name == mayerCost){
                    std::ostringstream errorMsg;
                    errorMsg << "Cannot change the TimeRange of cost named " << name << " since it is a terminal cost." << std::endl;
                    reportError("OptimalControlProblem", "updateCostTimeRange", errorMsg.str().c_str());
                }

            CostsMap::iterator costIterator;
            costIterator = m_pimpl->costs.find(name);
            if (costIterator == m_pimpl->costs.end()){
                std::ostringstream errorMsg;
                errorMsg << "Unable to find cost named "<<name<< std::endl;
                reportError("OptimalControlProblem", "updateCostTimeRange", errorMsg.str().c_str());
                return false;
            }

            costIterator->second.timeRange = newTimeRange;
            return true;
        }

        bool OptimalControlProblem::removeCost(const std::string &name)
        {
            if (m_pimpl->costs.erase(name)){
                for (auto mayerCost : m_pimpl->mayerCostnames) {
                    if (name == mayerCost) {
                        mayerCost.erase();
                    }
                }
                return true;
            }

            std::ostringstream errorMsg;
            errorMsg << "Failed to remove cost named "<<name<< std::endl;
            reportError("OptimalControlProblem", "removeCost", errorMsg.str().c_str());
            return false;
        }

        std::vector<TimeRange> &OptimalControlProblem::getCostsTimeRanges() const
        {
            if (m_pimpl->costTimeRanges.size() != m_pimpl->costs.size()) {
                m_pimpl->costTimeRanges.resize(m_pimpl->costs.size());
            }

            size_t i = 0;
            for (auto& c : m_pimpl->costs){
                m_pimpl->costTimeRanges[i] = c.second.timeRange;
                ++i;
            }
            return m_pimpl->costTimeRanges;
        }

        bool OptimalControlProblem::hasOnlyLinearCosts() const
        {
            for (auto& c : m_pimpl->costs) {
                if (!(c.second.isLinear)) {
                    return false;
                }
            }
            return true;
        }

        bool OptimalControlProblem::hasOnlyQuadraticCosts() const
        {
            for (auto& c : m_pimpl->costs) {
                if (!(c.second.isQuadratic)) {
                    return false;
                }
            }
            return true;
        }

        bool OptimalControlProblem::setStateLowerBound(const VectorDynSize &minState)
        {
            if (!(m_pimpl->dynamicalSystem)){
                reportError("OptimalControlProblem", "setStateLowerBound", "First a dynamical system has to be set.");
                return false;
            }

            if (minState.size() != m_pimpl->dynamicalSystem->stateSpaceSize()) {
                reportError("OptimalControlProblem", "setStateLowerBound", "The dimension of minState does not coincide with the state dimension.");
                return false;
            }
            m_pimpl->stateLowerBounded = true;
            m_pimpl->stateLowerBound = std::make_shared<TimeInvariantVector>(minState);

            return true;
        }

        bool OptimalControlProblem::setStateLowerBound(std::shared_ptr<TimeVaryingVector> minState)
        {
            if (!(m_pimpl->dynamicalSystem)){
                reportError("OptimalControlProblem", "setStateLowerBound", "First a dynamical system has to be set.");
                return false;
            }

            if (!minState) {
                reportError("OptimalControlProblem", "setStateLowerBound", "Empty minState pointer.");
                return false;
            }

            m_pimpl->stateLowerBounded = true;
            m_pimpl->stateLowerBound = minState;

            return true;
        }

        bool OptimalControlProblem::setStateUpperBound(const VectorDynSize &maxState)
        {
            if (!(m_pimpl->dynamicalSystem)){
                reportError("OptimalControlProblem", "setStateUpperBound", "First a dynamical system has to be set.");
                return false;
            }

            if (maxState.size() != m_pimpl->dynamicalSystem->stateSpaceSize()) {
                reportError("OptimalControlProblem", "setStateUpperBound", "The dimension of maxState does not coincide with the state dimension.");
                return false;
            }
            m_pimpl->stateUpperBounded = true;
            m_pimpl->stateUpperBound = std::make_shared<TimeInvariantVector>(maxState);

            return true;
        }

        bool OptimalControlProblem::setStateUpperBound(std::shared_ptr<TimeVaryingVector> maxState)
        {
            if (!(m_pimpl->dynamicalSystem)){
                reportError("OptimalControlProblem", "setStateUpperBound", "First a dynamical system has to be set.");
                return false;
            }

            if (!maxState) {
                reportError("OptimalControlProblem", "setStateUpperBound", "Empty maxState pointer.");
                return false;
            }

            m_pimpl->stateUpperBounded = true;
            m_pimpl->stateUpperBound = maxState;

            return true;
        }

        bool OptimalControlProblem::setControlLowerBound(const VectorDynSize &minControl)
        {
            if (!(m_pimpl->dynamicalSystem)){
                reportError("OptimalControlProblem", "setControlLowerBound", "First a dynamical system has to be set.");
                return false;
            }

            if (minControl.size() != m_pimpl->dynamicalSystem->controlSpaceSize()) {
                reportError("OptimalControlProblem", "setControlLowerBound", "The dimension of minControl does not coincide with the control dimension.");
                return false;
            }

            m_pimpl->controlLowerBounded = true;
            m_pimpl->controlLowerBound = std::make_shared<TimeInvariantVector>(minControl);

            return true;
        }

        bool OptimalControlProblem::setControlLowerBound(std::shared_ptr<TimeVaryingVector> minControl)
        {
            if (!(m_pimpl->dynamicalSystem)){
                reportError("OptimalControlProblem", "setControlLowerBound", "First a dynamical system has to be set.");
                return false;
            }

            if (!minControl) {
                reportError("OptimalControlProblem", "setControlLowerBound", "Empty minControl pointer.");
                return false;
            }

            m_pimpl->controlLowerBounded = true;
            m_pimpl->controlLowerBound = minControl;

            return true;
        }

        bool OptimalControlProblem::setControlUpperBound(const VectorDynSize &maxControl)
        {
            if (!(m_pimpl->dynamicalSystem)){
                reportError("OptimalControlProblem", "setControlUpperBound", "First a dynamical system has to be set.");
                return false;
            }

            if (maxControl.size() != m_pimpl->dynamicalSystem->controlSpaceSize()) {
                reportError("OptimalControlProblem", "setControlUpperBound", "The dimension of maxControl does not coincide with the control dimension.");
                return false;
            }

            m_pimpl->controlUpperBounded = true;
            m_pimpl->controlUpperBound = std::make_shared<TimeInvariantVector>(maxControl);

            return true;
        }

        bool OptimalControlProblem::setControlUpperBound(std::shared_ptr<TimeVaryingVector> maxControl)
        {
            if (!(m_pimpl->dynamicalSystem)){
                reportError("OptimalControlProblem", "setControlUpperBound", "First a dynamical system has to be set.");
                return false;
            }

            if (!maxControl) {
                reportError("OptimalControlProblem", "setControlUpperBound", "Empty maxControl pointer.");
                return false;
            }

            m_pimpl->controlUpperBounded = true;
            m_pimpl->controlUpperBound = maxControl;

            return true;
        }

        bool OptimalControlProblem::setStateBoxConstraints(const VectorDynSize &minState, const VectorDynSize &maxState)
        {
            return setStateLowerBound(minState) && setStateUpperBound(maxState);
        }

        bool OptimalControlProblem::setStateBoxConstraints(std::shared_ptr<TimeVaryingVector> minState, std::shared_ptr<TimeVaryingVector> maxState)
        {
            return setStateLowerBound(minState) && setStateUpperBound(maxState);
        }

        bool OptimalControlProblem::setControlBoxConstraints(const VectorDynSize &minControl, const VectorDynSize &maxControl)
        {
            return setControlLowerBound(minControl) && setControlUpperBound(maxControl);
        }

        bool OptimalControlProblem::setControlBoxConstraints(std::shared_ptr<TimeVaryingVector> minControl, std::shared_ptr<TimeVaryingVector> maxControl)
        {
            return setControlLowerBound(minControl) && setControlUpperBound(maxControl);
        }

        bool OptimalControlProblem::getStateLowerBound(double time, VectorDynSize &minState)
        {
            if (m_pimpl->stateLowerBounded){
                bool isValid = false;
                minState = m_pimpl->stateLowerBound->get(time, isValid);
                return isValid;
            }
            return false;
        }

        bool OptimalControlProblem::getStateUpperBound(double time, VectorDynSize &maxState)
        {
            if (m_pimpl->stateUpperBounded){
                bool isValid = false;
                maxState = m_pimpl->stateUpperBound->get(time, isValid);
                return isValid;
            }
            return false;
        }

        bool OptimalControlProblem::getControlLowerBound(double time, VectorDynSize &minControl)
        {
            if (m_pimpl->controlLowerBounded){
                bool isValid = false;
                minControl = m_pimpl->controlLowerBound->get(time, isValid);
                return true;
            }
            return false;
        }

        bool OptimalControlProblem::getControlUpperBound(double time, VectorDynSize &maxControl)
        {
            if (m_pimpl->controlUpperBounded){
                bool isValid = false;
                maxControl = m_pimpl->controlUpperBound->get(time, isValid);
                return true;
            }
            return false;
        }

        bool OptimalControlProblem::costsEvaluation(double time, const VectorDynSize &state, const VectorDynSize &control, double &costValue)
        {
            costValue = 0;
            double addCost;
            for(auto& cost : m_pimpl->costs){
                if (cost.second.timeRange.isInRange(time)){
                    if(!cost.second.cost->costEvaluation(time, state, control, addCost)){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost " << cost.second.cost->name() <<".";
                        reportError("OptimalControlProblem", "costsEvaluation", errorMsg.str().c_str());
                        return false;
                    }
                    costValue += cost.second.weight*addCost;
                }
            }

            return true;
        }

        bool OptimalControlProblem::costsFirstPartialDerivativeWRTState(double time, const VectorDynSize &state, const VectorDynSize &control, VectorDynSize &partialDerivative)
        {
            if (partialDerivative.size() != state.size()) {
                partialDerivative.resize(state.size());
            }

            if (m_pimpl->costStateJacobianBuffer.size() != state.size()) {
                m_pimpl->costStateJacobianBuffer.resize(state.size());
            }

            bool first = true;

            for(auto&  cost : m_pimpl->costs){
                if (cost.second.timeRange.isInRange(time)){
                    if(!cost.second.cost->costFirstPartialDerivativeWRTState(time, state, control, m_pimpl->costStateJacobianBuffer)){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost " << cost.second.cost->name() <<".";
                        reportError("OptimalControlProblem", "costsFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                        return false;
                    }
                    if (m_pimpl->costStateJacobianBuffer.size() != state.size()){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating " << cost.second.cost->name() <<": " << "the jacobian size is expected to match the state dimension.";
                        reportError("OptimalControlProblem", "costsFirstPartialDerivativeWRTState", errorMsg.str().c_str());
                        return false;
                    }
                    if (first){
                        toEigen(partialDerivative) = cost.second.weight * toEigen(m_pimpl->costStateJacobianBuffer);
                        first = false;
                    } else {
                        toEigen(partialDerivative) += cost.second.weight * toEigen(m_pimpl->costStateJacobianBuffer);
                    }
                }
            }
            return true;
        }

        bool OptimalControlProblem::costsFirstPartialDerivativeWRTControl(double time, const VectorDynSize &state, const VectorDynSize &control, VectorDynSize &partialDerivative)
        {
            if (partialDerivative.size() != control.size()) {
                partialDerivative.resize(control.size());
            }

            if (m_pimpl->costControlJacobianBuffer.size() != control.size()) {
                m_pimpl->costControlJacobianBuffer.resize(control.size());
            }

            bool first = true;

            for(auto& cost : m_pimpl->costs){
                if (cost.second.timeRange.isInRange(time)){
                    if (!cost.second.cost->costFirstPartialDerivativeWRTControl(time, state, control, m_pimpl->costControlJacobianBuffer)){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost " << cost.second.cost->name() <<".";
                        reportError("OptimalControlProblem", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                        return false;
                    }
                    if (m_pimpl->costControlJacobianBuffer.size() != control.size()){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating " << cost.second.cost->name() <<": " << "the jacobian size is expected to match the control dimension.";
                        reportError("OptimalControlProblem", "costFirstPartialDerivativeWRTControl", errorMsg.str().c_str());
                        return false;
                    }
                    if (first){
                        toEigen(partialDerivative) = cost.second.weight * toEigen(m_pimpl->costControlJacobianBuffer);
                        first = false;
                    } else {
                        toEigen(partialDerivative) += cost.second.weight * toEigen(m_pimpl->costControlJacobianBuffer);
                    }
                }
            }
            return true;
        }

        bool OptimalControlProblem::costsSecondPartialDerivativeWRTState(double time, const VectorDynSize &state, const VectorDynSize &control, MatrixDynSize &partialDerivative)
        {
            if ((partialDerivative.rows() != state.size()) || (partialDerivative.cols() != state.size())) {
                partialDerivative.resize(state.size(), state.size());
            }

            if ((m_pimpl->costStateHessianBuffer.rows() != state.size()) || (m_pimpl->costStateHessianBuffer.cols() != state.size())) {
                m_pimpl->costStateHessianBuffer.resize(state.size(), state.size());
            }

            bool first = true;

            for (auto& cost : m_pimpl->costs){
                if (cost.second.timeRange.isInRange(time)){
                    if (!cost.second.cost->costSecondPartialDerivativeWRTState(time, state, control, m_pimpl->costStateHessianBuffer)){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost " << cost.second.cost->name() <<".";
                        reportError("OptimalControlProblem", "costSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                        return false;
                    }

                    if ((m_pimpl->costStateHessianBuffer.rows() != state.size()) || (m_pimpl->costStateHessianBuffer.cols() != state.size())){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating " << cost.second.cost->name() <<": " << "the hessian size is expected to be a square matrix matching the state dimension.";
                        reportError("OptimalControlProblem", "costSecondPartialDerivativeWRTState", errorMsg.str().c_str());
                        return false;
                    }
                    if (first){
                        toEigen(partialDerivative) = cost.second.weight * toEigen(m_pimpl->costStateHessianBuffer);
                        first = false;
                    } else {
                        toEigen(partialDerivative) += cost.second.weight * toEigen(m_pimpl->costStateHessianBuffer);
                    }
                }
            }
            return true;
        }

        bool OptimalControlProblem::costsSecondPartialDerivativeWRTControl(double time, const VectorDynSize &state, const VectorDynSize &control, MatrixDynSize &partialDerivative)
        {
            if ((partialDerivative.rows() != control.size()) || (partialDerivative.cols() != control.size())) {
                partialDerivative.resize(control.size(), control.size());
            }

            if ((m_pimpl->costControlHessianBuffer.rows() != control.size()) || (m_pimpl->costControlHessianBuffer.cols() != control.size())) {
                m_pimpl->costControlHessianBuffer.resize(control.size(), control.size());
            }

            bool first = true;

            for (auto& cost : m_pimpl->costs){
                if (cost.second.timeRange.isInRange(time)){
                    if (!cost.second.cost->costSecondPartialDerivativeWRTControl(time, state, control, m_pimpl->costControlHessianBuffer)){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost " << cost.second.cost->name() <<".";
                        reportError("OptimalControlProblem", "costSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                        return false;
                    }

                    if ((m_pimpl->costControlHessianBuffer.rows() != control.size()) || (m_pimpl->costControlHessianBuffer.cols() != control.size())){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating " << cost.second.cost->name() <<": " << "the hessian size is expected to be a square matrix matching the control dimension.";
                        reportError("OptimalControlProblem", "costSecondPartialDerivativeWRTControl", errorMsg.str().c_str());
                        return false;
                    }

                    if (first){
                        toEigen(partialDerivative) = cost.second.weight * toEigen(m_pimpl->costControlHessianBuffer);
                        first = false;
                    } else {
                        toEigen(partialDerivative) += cost.second.weight * toEigen(m_pimpl->costControlHessianBuffer);
                    }
                }
            }
            return true;
        }

        bool OptimalControlProblem::costsSecondPartialDerivativeWRTStateControl(double time, const VectorDynSize &state, const VectorDynSize &control, MatrixDynSize &partialDerivative)
        {
            if ((partialDerivative.rows() != state.size()) || (partialDerivative.cols() != control.size())) {
                partialDerivative.resize(state.size(), control.size());
            }

            if ((m_pimpl->costMixedHessianBuffer.rows() != state.size()) || (m_pimpl->costMixedHessianBuffer.cols() != control.size())) {
                m_pimpl->costMixedHessianBuffer.resize(state.size(), control.size());
            }

            bool first = true;

            for (auto& cost : m_pimpl->costs){
                if (cost.second.timeRange.isInRange(time)){
                    if (!cost.second.cost->costSecondPartialDerivativeWRTStateControl(time, state, control, m_pimpl->costMixedHessianBuffer)){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost " << cost.second.cost->name() <<".";
                        reportError("OptimalControlProblem", "costSecondPartialDerivativeWRTStateControl", errorMsg.str().c_str());
                        return false;
                    }

                    if ((m_pimpl->costMixedHessianBuffer.rows() != state.size()) || (m_pimpl->costMixedHessianBuffer.cols() != control.size())){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating " << cost.second.cost->name() <<": " << "the hessian size is expected to have as many rows as the state dimension and a number of columns matching the control dimension.";
                        reportError("OptimalControlProblem", "costSecondPartialDerivativeWRTStateControl", errorMsg.str().c_str());
                        return false;
                    }

                    if (first){
                        toEigen(partialDerivative) = cost.second.weight * toEigen(m_pimpl->costMixedHessianBuffer);
                        first = false;
                    } else {
                        toEigen(partialDerivative) += cost.second.weight * toEigen(m_pimpl->costMixedHessianBuffer);
                    }
                }
            }
            return true;
        }

        bool OptimalControlProblem::constraintsEvaluation(double time, const VectorDynSize &state, const VectorDynSize &control, VectorDynSize &constraintsValue)
        {
            if (constraintsValue.size() != getConstraintsDimension()) {
                constraintsValue.resize(getConstraintsDimension());
            }

            Eigen::Map< Eigen::VectorXd > constraintsEvaluation = toEigen(constraintsValue);
            Eigen::Index offset = 0;

            for (auto& group : m_pimpl->constraintsGroups){
                if (!(group.second->group_ptr->evaluateConstraints(time, state, control, group.second->constraintsBuffer))){
                    std::ostringstream errorMsg;
                    errorMsg << "Error while evaluating constraint " << group.second->group_ptr->name() <<".";
                    reportError("OptimalControlProblem", "constraintsEvaluation", errorMsg.str().c_str());
                    return false;
                }

                constraintsEvaluation.segment(offset, group.second->constraintsBuffer.size()) = toEigen(group.second->constraintsBuffer);
                offset += group.second->constraintsBuffer.size();
            }

            return true;
        }

        bool OptimalControlProblem::getConstraintsUpperBound(double time, double infinity, VectorDynSize &upperBound)
        {
            if (upperBound.size() != getConstraintsDimension()) {
                upperBound.resize(getConstraintsDimension());
            }

            Eigen::Map< Eigen::VectorXd > upperBoundMap = toEigen(upperBound);
            Eigen::Index offset = 0;

            for (auto& group : m_pimpl->constraintsGroups){
                if (! group.second->group_ptr->getUpperBound(time, group.second->constraintsBuffer)){
                    toEigen(group.second->constraintsBuffer).setConstant(std::abs(infinity)); //if not upper bounded
                }

                if (group.second->constraintsBuffer.size() != group.second->group_ptr->constraintsDimension()){
                    std::ostringstream errorMsg;
                    errorMsg << "Upper bound dimension different from dimension of group " << group.second->group_ptr->name() << ".";
                    reportError("OptimalControlProblem", "getConstraintsUpperBound", errorMsg.str().c_str());
                    return false;
                }

                upperBoundMap.segment(offset, group.second->constraintsBuffer.size()) = toEigen(group.second->constraintsBuffer);
                offset += group.second->constraintsBuffer.size();
            }

            return true;
        }

        bool OptimalControlProblem::getConstraintsLowerBound(double time, double infinity, VectorDynSize &lowerBound)
        {
            if (lowerBound.size() != getConstraintsDimension()) {
                lowerBound.resize(getConstraintsDimension());
            }

            Eigen::Map< Eigen::VectorXd > lowerBoundMap = toEigen(lowerBound);
            Eigen::Index offset = 0;

            for (auto& group : m_pimpl->constraintsGroups){
                if (! group.second->group_ptr->getLowerBound(time, group.second->constraintsBuffer)){
                    toEigen(group.second->constraintsBuffer).setConstant(-std::abs(infinity)); //if not lower bounded
                }

                if (group.second->constraintsBuffer.size() != group.second->group_ptr->constraintsDimension()){
                    std::ostringstream errorMsg;
                    errorMsg << "Lower bound dimension different from dimension of group " << group.second->group_ptr->name() << ".";
                    reportError("OptimalControlProblem", "getConstraintsUpperBound", errorMsg.str().c_str());
                    return false;
                }

                lowerBoundMap.segment(offset, group.second->constraintsBuffer.size()) = toEigen(group.second->constraintsBuffer);
                offset += group.second->constraintsBuffer.size();
            }

            return true;
        }

        bool OptimalControlProblem::isFeasiblePoint(double time, const VectorDynSize &state, const VectorDynSize &control)
        {
            for(auto& group : m_pimpl->constraintsGroups){
                if(!(group.second->group_ptr->isFeasibilePoint(time, state, control))){
                    return false;
                }
            }
            return true;
        }

        bool OptimalControlProblem::constraintsJacobianWRTState(double time, const VectorDynSize &state, const VectorDynSize &control, MatrixDynSize &jacobian)
        {
            unsigned int nc = getConstraintsDimension();
            if ((jacobian.rows() != nc) || (jacobian.cols() != state.size())) {
                jacobian.resize(nc, state.size());
            }

            iDynTreeEigenMatrixMap jacobianMap = toEigen(jacobian);
            Eigen::Index offset = 0;

            for (auto& group : m_pimpl->constraintsGroups){
                if (!(group.second->group_ptr->constraintJacobianWRTState(time, state, control, group.second->stateJacobianBuffer))){
                    std::ostringstream errorMsg;
                    errorMsg << "Error while evaluating constraint group " << group.second->group_ptr->name() <<".";
                    reportError("OptimalControlProblem", "constraintsJacobianWRTState", errorMsg.str().c_str());
                    return false;
                }

                jacobianMap.block(offset, 0, group.second->group_ptr->constraintsDimension(), state.size()) = toEigen(group.second->stateJacobianBuffer);
                offset += group.second->stateJacobianBuffer.rows();
            }
            return true;
        }

        bool OptimalControlProblem::constraintsJacobianWRTControl(double time, const VectorDynSize &state, const VectorDynSize &control, MatrixDynSize &jacobian)
        {
            unsigned int nc = getConstraintsDimension();
            if ((jacobian.rows() != nc) || (jacobian.cols() != control.size())) {
                jacobian.resize(nc, control.size());
            }

            iDynTreeEigenMatrixMap jacobianMap = toEigen(jacobian);
            Eigen::Index offset = 0;

            for (auto& group : m_pimpl->constraintsGroups){
                if (! group.second->group_ptr->constraintJacobianWRTControl(time, state, control, group.second->controlJacobianBuffer)){
                    std::ostringstream errorMsg;
                    errorMsg << "Error while evaluating constraint " << group.second->group_ptr->name() <<".";
                    reportError("OptimalControlProblem", "constraintsJacobianWRTControl", errorMsg.str().c_str());
                    return false;
                }
                jacobianMap.block(offset, 0, group.second->group_ptr->constraintsDimension(), control.size()) = toEigen(group.second->controlJacobianBuffer);
                offset += group.second->controlJacobianBuffer.rows();
            }
            return true;
        }

        bool OptimalControlProblem::constraintJacobianWRTStateSparsity(SparsityStructure &stateSparsity)
        {
            size_t nonZeroIndexState = 0;
            size_t offset = 0;
            for (auto& group : m_pimpl->constraintsGroups){

                group.second->hasStateSparsity = group.second->group_ptr->constraintJacobianWRTStateSparsity(group.second->stateSparsity);

                if (group.second->hasStateSparsity) {

                    if (m_pimpl->stateSparsity.size() < (nonZeroIndexState + group.second->stateSparsity.size())) {
                        m_pimpl->stateSparsity.resize(nonZeroIndexState + group.second->stateSparsity.size());
                    }

                    for (size_t i = 0; i < group.second->stateSparsity.size(); ++i) {
                        m_pimpl->stateSparsity.nonZeroElementRows[nonZeroIndexState]    = group.second->stateSparsity.nonZeroElementRows[i] + offset;
                        m_pimpl->stateSparsity.nonZeroElementColumns[nonZeroIndexState] = group.second->stateSparsity.nonZeroElementColumns[i];
                        nonZeroIndexState++;
                    }

                } else {

                    if (m_pimpl->dynamicalSystem) {

                        size_t rows = group.second->stateJacobianBuffer.rows();
                        size_t cols = group.second->stateJacobianBuffer.cols();
                        size_t nonZeros = rows * cols;

                        if (m_pimpl->stateSparsity.size() < (nonZeroIndexState + nonZeros)) {
                            m_pimpl->stateSparsity.resize(nonZeroIndexState + nonZeros);
                        }

                        for (size_t i = 0; i < rows; ++i) {
                            for (size_t j = 0; j < cols; ++j) {
                                m_pimpl->stateSparsity.nonZeroElementRows[nonZeroIndexState]    = i + offset;
                                m_pimpl->stateSparsity.nonZeroElementColumns[nonZeroIndexState] = j;
                                nonZeroIndexState++;
                            }
                        }
                    } else {
                        std::ostringstream errorMsg;
                        errorMsg << "The group " << group.second->group_ptr->name() <<" has dense state jacobian and no dynamical system has been provided yet. Cannot determine the sparsity since the state dimension is unknown.";
                        reportError("OptimalControlProblem", "constraintJacobianWRTStateSparsity", errorMsg.str().c_str());
                        return false;
                    }
                }

                offset += group.second->stateJacobianBuffer.rows();
            }
            m_pimpl->stateSparsity.resize(nonZeroIndexState); //remove leftovers


            stateSparsity = m_pimpl->stateSparsity;

            return true;
        }

        bool OptimalControlProblem::constraintJacobianWRTControlSparsity(SparsityStructure &controlSparsity)
        {
            size_t nonZeroIndexControl = 0;
            size_t offset = 0;
            for (auto& group : m_pimpl->constraintsGroups){

                group.second->hasControlSparsity = group.second->group_ptr->constraintJacobianWRTControlSparsity(group.second->controlSparsity);

                if (group.second->hasControlSparsity) {

                    if (m_pimpl->controlSparsity.size() < (nonZeroIndexControl + group.second->controlSparsity.size())) {
                        m_pimpl->controlSparsity.resize(nonZeroIndexControl + group.second->controlSparsity.size());
                    }

                    for (size_t i = 0; i < group.second->controlSparsity.size(); ++i) {
                        m_pimpl->controlSparsity.nonZeroElementRows[nonZeroIndexControl]    = group.second->controlSparsity.nonZeroElementRows[i] + offset;
                        m_pimpl->controlSparsity.nonZeroElementColumns[nonZeroIndexControl] = group.second->controlSparsity.nonZeroElementColumns[i];
                        nonZeroIndexControl++;
                    }

                } else {

                    if (m_pimpl->dynamicalSystem) {

                        size_t rows = group.second->controlJacobianBuffer.rows();
                        size_t cols = group.second->controlJacobianBuffer.cols();
                        size_t nonZeros = rows * cols;

                        if (m_pimpl->controlSparsity.size() < (nonZeroIndexControl + nonZeros)) {
                            m_pimpl->controlSparsity.resize(nonZeroIndexControl + nonZeros);
                        }

                        for (size_t i = 0; i < rows; ++i) {
                            for (size_t j = 0; j < cols; ++j) {
                                m_pimpl->controlSparsity.nonZeroElementRows[nonZeroIndexControl]    = i + offset;
                                m_pimpl->controlSparsity.nonZeroElementColumns[nonZeroIndexControl] = j;
                                nonZeroIndexControl++;
                            }
                        }
                    } else {
                        std::ostringstream errorMsg;
                        errorMsg << "The group " << group.second->group_ptr->name() <<" has dense control jacobian and no dynamical system has been provided yet. Cannot determine the sparsity since the control dimension is unknown.";
                        reportError("OptimalControlProblem", "constraintJacobianWRTControlSparsity", errorMsg.str().c_str());
                        return false;
                    }
                }

                offset += group.second->controlJacobianBuffer.rows();
            }
            m_pimpl->controlSparsity.resize(nonZeroIndexControl); //remove leftovers


            controlSparsity = m_pimpl->controlSparsity;

            return true;
        }

    }
}
