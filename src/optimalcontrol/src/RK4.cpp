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

#include <iDynTree/Integrators/RK4.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree {
    namespace optimalcontrol {
        namespace integrators {
            RK4::RK4(const std::shared_ptr<iDynTree::optimalcontrol::DynamicalSystem> dynamicalSystem) : FixedStepIntegrator(dynamicalSystem){
                m_infoData->isExplicit = true;
                m_infoData->numberOfStages = 4;
                m_infoData->name = "RK4";

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

                allocateBuffers();
            }

            RK4::~RK4(){
            }

            bool RK4::oneStepIntegration(double t0, double dT, const VectorDynSize &x0, VectorDynSize &x){
                if (!m_dynamicalSystem_ptr){
                    reportError(m_info.name().c_str(), "oneStepIntegration", "Dynamical system not set.");
                }

                if (x.size() != m_dynamicalSystem_ptr->stateSpaceSize()) {
                    x.resize(m_dynamicalSystem_ptr->stateSpaceSize());
                }

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

            bool RK4::allocateBuffers()
            {
                if (!m_dynamicalSystem_ptr) {
                    return false;
                }

                m_K.resize(m_dynamicalSystem_ptr->stateSpaceSize(), 4);

                m_computationBuffer.resize(static_cast<unsigned int>(m_dynamicalSystem_ptr->stateSpaceSize()));

                return true;
            }

            RK4::RK4()
            {
                m_infoData->isExplicit = true;
                m_infoData->numberOfStages = 4;
                m_infoData->name = "RK4";

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
            }
        }
    }
}
