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

#ifndef IDYNTREE_OPTIMALCONTROL_QUADRATICCOST_H
#define IDYNTREE_OPTIMALCONTROL_QUADRATICCOST_H

#include <iDynTree/Cost.h>
#include <iDynTree/TimeVaryingObject.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <memory>

namespace iDynTree {
    namespace optimalcontrol {

       /**
        * @warning This class is still in active development, and so API interface can change between iDynTree versions.
        * \ingroup iDynTreeExperimental
        */

       class QuadraticCost
       : public Cost {

       public:
           QuadraticCost(const std::string& costName);

           virtual ~QuadraticCost() override;

           bool setStateCost(const iDynTree::MatrixDynSize& stateHessian, const iDynTree::VectorDynSize& stateGradient);

           bool setStateCost(std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingMatrix> timeVaryingStateHessian,
                             std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingVector> timeVaryingStateGradient);

           bool setControlCost(const iDynTree::MatrixDynSize& controlHessian, const iDynTree::VectorDynSize& controlGradient);

           bool setControlCost(std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingMatrix> timeVaryingControlHessian,
                               std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingVector> timeVaryingControlGradient);

           virtual bool costEvaluation(double time,
                                       const iDynTree::VectorDynSize& state,
                                       const iDynTree::VectorDynSize& control,
                                       double& costValue) override;//0.5 xT H x + gx

           virtual bool costFirstPartialDerivativeWRTState(double time,
                                                           const iDynTree::VectorDynSize& state,
                                                           const iDynTree::VectorDynSize& control,
                                                           iDynTree::VectorDynSize& partialDerivative) override;

           virtual bool costFirstPartialDerivativeWRTControl(double time,
                                                             const iDynTree::VectorDynSize& state,
                                                             const iDynTree::VectorDynSize& control,
                                                             iDynTree::VectorDynSize& partialDerivative) override;

           virtual bool costSecondPartialDerivativeWRTState(double time,
                                                            const iDynTree::VectorDynSize& state,
                                                            const iDynTree::VectorDynSize& control,
                                                            iDynTree::MatrixDynSize& partialDerivative) override;

           virtual bool costSecondPartialDerivativeWRTControl(double time,
                                                              const iDynTree::VectorDynSize& state,
                                                              const iDynTree::VectorDynSize& control,
                                                              iDynTree::MatrixDynSize& partialDerivative) override;


           virtual bool costSecondPartialDerivativeWRTStateControl(double time,
                                                                   const iDynTree::VectorDynSize& state,
                                                                   const iDynTree::VectorDynSize& control,
                                                                   iDynTree::MatrixDynSize& partialDerivative) override;

       protected:
           std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingMatrix> m_timeVaryingStateHessian;
           std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingVector> m_timeVaryingStateGradient;
           std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingMatrix> m_timeVaryingControlHessian;
           std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingVector> m_timeVaryingControlGradient;
           double m_stateCostScale, m_controlCostScale;
       };
    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_QUADRATICCOST_H */
