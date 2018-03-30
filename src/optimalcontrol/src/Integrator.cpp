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

#include "iDynTree/Integrator.h"
#include "iDynTree/DynamicalSystem.h"
#include "iDynTree/Core/Utils.h"
#include "iDynTree/Core/EigenHelpers.h"

#include <cassert>
#include <cstddef>
#include <sstream>

namespace iDynTree {
    namespace optimalcontrol {
        namespace integrators {

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

            const std::weak_ptr<const DynamicalSystem> Integrator::dynamicalSystem() const{
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

                for (std::vector<solutionElement>::const_iterator iteration = m_solution.cbegin();
                     (iteration +1) != m_solution.cend(); ++iteration){

                    if ((iteration->time <= time) && ((iteration + 1)->time >= time)){
                        return interpolatePoints(iteration, iteration+1, time, solution);
                    }
                }

                reportError(m_info.name().c_str(), "getSolution", "Error while searching the desired time.");
                return false;
            }

            const std::vector<solutionElement> &Integrator::getFullSolution() const{
                return m_solution;
            }

            void Integrator::clearSolution()
            {
                m_solution.clear();
            }

            bool Integrator::evaluateCollocationConstraint(const std::vector<VectorDynSize> &collocationPoints,
                                                           const std::vector<VectorDynSize> &controlInputs,
                                                           double time, VectorDynSize &constraintValue)
            {
                return false;
            }

            bool Integrator::evaluateCollocationConstraintJacobian(const std::vector<VectorDynSize> &collocationPoints,
                                                                   const std::vector<VectorDynSize> &controlInputs,
                                                                   double time,
                                                                   std::vector<MatrixDynSize> &stateJacobianValues,
                                                                   std::vector<MatrixDynSize> &controlJacobianValues)
            {
                return false;
            }

            const IntegratorInfo &Integrator::info() const
            {
                return m_info;
            }

            bool Integrator::interpolatePoints(const std::vector<solutionElement>::const_iterator &first, const std::vector<solutionElement>::const_iterator &second, double time, VectorDynSize &outputPoint) const{

                double ratio = (second->time - time)/(second->time - first->time);

                if(outputPoint.size() != first->stateAtT.size()){
                    outputPoint.resize(first->stateAtT.size());
                }
                toEigen(outputPoint) = ratio * toEigen(first->stateAtT) + (1-ratio) * toEigen(second->stateAtT);

                return true;
            }

        }

    }
}
