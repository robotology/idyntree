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

#include <iDynTree/Integrators/FixedStepIntegrator.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Utils.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <string>


namespace iDynTree {
    namespace optimalcontrol {
    namespace integrators {

            FixedStepIntegrator::FixedStepIntegrator(const std::shared_ptr<DynamicalSystem> dynamicalSystem) : Integrator(dynamicalSystem){
                m_infoData->name = "FixedStepIntegrator";
            }

            FixedStepIntegrator::FixedStepIntegrator()
            {
                m_infoData->name = "FixedStepIntegrator";
            }

            FixedStepIntegrator::~FixedStepIntegrator(){
            }

            bool FixedStepIntegrator::integrate(double initialTime, double finalTime){

                if (!m_dynamicalSystem_ptr){
                    reportError(m_info.name().c_str(), "integrate", "No dynamical system have been set yet.");
                    return false;
                }

                if ((finalTime - initialTime) < 0){
                    reportError(m_info.name().c_str(), "integrate", "The final time is supposed to be greater than the initial time.");
                    return false;
                }
                if (m_dTmax <= 0){
                    reportError(m_info.name().c_str(), "integrate", "The maximum dT should be greater than zero.");
                    return false;
                }

                unsigned int stateDim = static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize());

                int iterations = std::ceil((finalTime - initialTime)/m_dTmax);

                m_solution.resize(iterations+1);

                if(m_dynamicalSystem_ptr->initialState().size() != stateDim){
                    reportError(m_info.name().c_str(), "integrate", "The initial state has a wrong dimension.");
                    return false;
                }

                m_solution[0].stateAtT.resize(stateDim);
                m_solution[0].stateAtT = m_dynamicalSystem_ptr->initialState();
                m_solution[0].time = initialTime;

                double dT = 0;
                if (iterations > 0){
                    dT = (finalTime - initialTime)/iterations;
                }

                for(int i = 0; i < (iterations - 1); ++i){
                    if (!oneStepIntegration(initialTime + dT*i, dT, m_solution[i].stateAtT, m_solution[i+1].stateAtT)){
                        std::ostringstream errorMsg;
                        errorMsg << "Error at time " << initialTime + dT*i << ".";
                        reportError(m_info.name().c_str(), "integrate", errorMsg.str().c_str());
                        return false;
                    }
                    m_solution[i+1].time = initialTime + dT*(i+1);
                }
                //Consider last step separately to be sure that the last solution point is in finalTime
                dT = finalTime - m_solution[iterations - 1].time;
                if (!oneStepIntegration(m_solution[iterations - 1].time, dT, m_solution[iterations - 1].stateAtT, m_solution.back().stateAtT)){
                    std::ostringstream errorMsg;
                    errorMsg << "Error in last iteration";
                    reportError(m_info.name().c_str(), "integrate", errorMsg.str().c_str());
                    return false;
                }
                m_solution.back().time = finalTime;

                return true;
            }
        }
    }
}
