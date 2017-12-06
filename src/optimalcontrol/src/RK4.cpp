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

#include "Integrators/RK4.h"
#include "DynamicalSystem.h"
#include "iDynTree/Core/EigenHelpers.h"
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <string>

namespace iDynTree {
    namespace optimalcontrol {
        namespace integrators {
            RK4::RK4(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem) : Integrator(dynamicalSystem){
                m_aCoefficents.resize(4,4);
                m_aCoefficents.reserve(6);
                m_aCoefficents.insert(1,0) =  1.0/3.0;
                m_aCoefficents.insert(2,0) = -1.0/3.0;
                m_aCoefficents.insert(2,1) =   1.0;
                m_aCoefficents.insert(3,0) =   1.0;
                m_aCoefficents.insert(3,1) =  -1.0;
                m_aCoefficents.insert(3,2) =   1.0;

                m_bCoefficients.resize(4);
                m_bCoefficients(0) = 1.0/8.0;
                m_bCoefficients(1) = 3.0/8.0;
                m_bCoefficients(2) = 3.0/8.0;
                m_bCoefficients(3) = 1.0/8.0;
                m_cCoefficients.resize(4);
                m_cCoefficients(0) = 0.0;
                m_cCoefficients(1) = 1.0/3.0;
                m_cCoefficients(2) = 2.0/3.0;
                m_cCoefficients(3) = 1.0;

                m_K.resize(m_dynamicalSystem_ptr->stateSpaceSize(), 4);

                m_computationBuffer.resize(static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize()));
            }

            RK4::~RK4(){
            }

            bool RK4::oneStepIntegration(double t0, double dT, const VectorDynSize &x0, VectorDynSize &x){
                if (x.size() != m_dynamicalSystem_ptr->stateSpaceSize())
                    x.resize(m_dynamicalSystem_ptr->stateSpaceSize());

                Eigen::Map<Eigen::VectorXd> x_map(x.data(), x.size());
                const Eigen::Map<const Eigen::VectorXd> x0_map(x0.data(), x0.size());
                Eigen::Map<Eigen::VectorXd> buffer_map(m_computationBuffer.data(), m_computationBuffer.size());
                bool ok;
                for (int i=0; i<4; ++i){
                    buffer_map = m_K * m_aCoefficents.row(i).transpose(); //use the x vector as buffer
                    buffer_map = dT*buffer_map;   //use the x vector as buffer
                    x_map = x0_map + buffer_map;
                    ok = m_dynamicalSystem_ptr->dynamics(x, t0 + m_cCoefficients(i)*dT, m_computationBuffer);
                    if(!ok){
                        return false;
                    }
                    m_K.col(i) = buffer_map;
                }

                buffer_map = m_K * m_bCoefficients;
                x_map = x0_map + dT * buffer_map;
                return true;
            }

            bool RK4::integrate(double initialTime, double finalTime){
                if ((finalTime - initialTime) < 0){
                    reportError("RK4", "integrate", "The final time is supposed to be greater than the initial time.");
                    return false;
                }
                if (m_dTmax <= 0){
                    reportError("RK4", "integrate", "The maximum dT should be greater than zero.");
                    return false;
                }

                unsigned int stateDim = static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize());

                int iterations = std::ceil((finalTime - initialTime)/m_dTmax);

                m_solution.resize(iterations+1);

                if(m_dynamicalSystem_ptr->initialState().size() != stateDim){
                    reportError("RK4", "integrate", "The initial state has a wrong dimension.");
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
                        reportError("RK4", "integrate", errorMsg.str().c_str());
                        return false;
                    }
                    m_solution[i+1].time = initialTime + dT*(i+1);
                }
                //Consider last step separately to be sure that the last solution point is in finalTime
                dT = finalTime - m_solution[iterations - 1].time;
                if (!oneStepIntegration(m_solution[iterations - 1].time, dT, m_solution[iterations - 1].stateAtT, m_solution.back().stateAtT)){
                    std::ostringstream errorMsg;
                    errorMsg << "Error in last iteration";
                    reportError("RK4", "integrate", errorMsg.str().c_str());
                    return false;
                }
                m_solution.back().time = finalTime;

                return true;
            }
        }
    }
}
