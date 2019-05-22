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

#include <iDynTree/Integrator.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <cassert>
#include <cstddef>
#include <sstream>

namespace iDynTree {
    namespace optimalcontrol {
        namespace integrators {

            Integrator::Integrator()
            : m_dTmax(0)
            , m_dynamicalSystem_ptr(nullptr)
            , m_infoData(new IntegratorInfoData)
            , m_info(m_infoData)
            {
            }

            Integrator::Integrator(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem)
            : m_dTmax(0)
            , m_dynamicalSystem_ptr(dynamicalSystem)
            , m_infoData(new IntegratorInfoData)
            , m_info(m_infoData)
            {
            }


            Integrator::~Integrator(){
            }


            bool Integrator::setMaximumStepSize(const double dT){
                if (dT <= 0){
                    reportError(m_info.name().c_str(), "setMaximumStepSize", "The dT must be positive.");
                    return false;
                }

               m_dTmax = dT;
                return true;
            }

            double Integrator::maximumStepSize() const{
                return m_dTmax;
            }

            bool Integrator::setDynamicalSystem(const std::shared_ptr<DynamicalSystem> dynamicalSystem)
            {
                if (m_dynamicalSystem_ptr){
                    reportError(m_info.name().c_str(), "setDynamicalSystem", "Change dynamical system is forbidden."); //I want to prevent the change of dynamical system between two iterations of an eventual optimal control problem. This change would not be detected and would cause problems of dimensions.
                    return false;
                }

                m_dynamicalSystem_ptr = dynamicalSystem;

                return allocateBuffers();
            }

            const std::weak_ptr<DynamicalSystem> Integrator::dynamicalSystem() const{
                return m_dynamicalSystem_ptr;
            }


            bool Integrator::getSolution(double time, VectorDynSize &solution) const{
                if (m_solution.size() == 0){
                    reportError(m_info.name().c_str(), "getSolution", "No solution computed yet.");
                    return false;
                }

                if((time < m_solution.front().time)||(time > m_solution.back().time)){
                    std::ostringstream errorMsg;
                    errorMsg << "Time outside the computed range. ";
                    errorMsg << "Valid range: [" << m_solution.front().time << ", " << m_solution.back().time << "]. ";
                    errorMsg << "Requested value: " << time << ".";
                    reportError(m_info.name().c_str(), "getSolution", errorMsg.str().c_str());
                    return false;
                }

                for (std::vector<SolutionElement>::const_iterator iteration = m_solution.cbegin();
                     (iteration +1) != m_solution.cend(); ++iteration){

                    if ((iteration->time <= time) && ((iteration + 1)->time >= time)){
                        return interpolatePoints(iteration, iteration+1, time, solution);
                    }
                }

                reportError(m_info.name().c_str(), "getSolution", "Error while searching the desired time.");
                return false;
            }

            const std::vector<SolutionElement> &Integrator::getFullSolution() const{
                return m_solution;
            }

            void Integrator::clearSolution()
            {
                m_solution.clear();
            }

            bool Integrator::evaluateCollocationConstraint(double /*time*/, const std::vector<VectorDynSize> &/*collocationPoints*/,
                                                           const std::vector<VectorDynSize> &/*controlInputs*/, double /*dT*/, VectorDynSize &/*constraintValue*/)
            {
                return false;
            }

            bool Integrator::evaluateCollocationConstraintJacobian(double /*time*/, const std::vector<VectorDynSize> &/*collocationPoints*/,
                                                                   const std::vector<VectorDynSize> &/*controlInputs*/, double /*dT*/,
                                                                   std::vector<MatrixDynSize> &/*stateJacobianValues*/,
                                                                   std::vector<MatrixDynSize> &/*controlJacobianValues*/)
            {
                return false;
            }

            bool Integrator::getCollocationConstraintJacobianStateSparsity(std::vector<SparsityStructure> &/*stateJacobianSparsity*/)
            {
                return false;
            }

            bool Integrator::getCollocationConstraintJacobianControlSparsity(std::vector<SparsityStructure> &/*controlJacobianSparsity*/)
            {
                return false;
            }

            bool iDynTree::optimalcontrol::integrators::Integrator::evaluateCollocationConstraintSecondDerivatives(double /*time*/,
                                                                                                                   const std::vector<VectorDynSize> &/*collocationPoints*/,
                                                                                                                   const std::vector<VectorDynSize> &/*controlInputs*/,
                                                                                                                   double /*dT*/, const VectorDynSize &/*lambda*/,
                                                                                                                   CollocationHessianMap &/*stateSecondDerivative*/,
                                                                                                                   CollocationHessianMap &/*controlSecondDerivative*/,
                                                                                                                   CollocationHessianMap &/*stateControlSecondDerivative*/)
            {
                return false;
            }

            bool Integrator::getCollocationConstraintSecondDerivativeWRTStateSparsity(CollocationHessianSparsityMap &/*stateDerivativeSparsity*/)
            {
                return false;
            }

            bool Integrator::getCollocationConstraintSecondDerivativeWRTControlSparsity(CollocationHessianSparsityMap &/*controlDerivativeSparsity*/)
            {
                return false;
            }

            bool Integrator::getCollocationConstraintSecondDerivativeWRTStateControlSparsity(CollocationHessianSparsityMap &/*stateControlDerivativeSparsity*/)
            {
                return false;
            }

            const IntegratorInfo &Integrator::info() const
            {
                return m_info;
            }

            bool Integrator::interpolatePoints(const std::vector<SolutionElement>::const_iterator &first, const std::vector<SolutionElement>::const_iterator &second, double time, VectorDynSize &outputPoint) const{

                double ratio = (second->time - time)/(second->time - first->time);

                if(outputPoint.size() != first->stateAtT.size()){
                    outputPoint.resize(first->stateAtT.size());
                }
                toEigen(outputPoint) = ratio * toEigen(first->stateAtT) + (1-ratio) * toEigen(second->stateAtT);

                return true;
            }

            bool Integrator::allocateBuffers()
            {
                return true;
            }

            IntegratorInfoData::IntegratorInfoData():name("Integrator"),isExplicit(true),numberOfStages(1){}

            IntegratorInfo::IntegratorInfo(std::shared_ptr<IntegratorInfoData> data):m_data(data){}

            const std::string &IntegratorInfo::name() const {return m_data->name;}

            bool IntegratorInfo::isExplicit() const {return m_data->isExplicit;}

            size_t IntegratorInfo::numberOfStages() const {return m_data->numberOfStages;}

            CollocationHessianIndex::CollocationHessianIndex(size_t first, size_t second)
                : m_first(first)
                  , m_second(second)
            { }

            bool CollocationHessianIndex::operator< (const CollocationHessianIndex& rhs) const {

                return (m_first < rhs.m_first) || ((m_first == rhs.m_first) && (m_second < rhs.m_second));
            }

            size_t CollocationHessianIndex::first() const
            {
                return m_first;
            }

            size_t CollocationHessianIndex::second() const
            {
                return m_second;
            }

        }

    }
}
