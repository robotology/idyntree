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

#ifndef IDYNTREE_OPTIMALCONTROL_QUADRATICLIKECOST_H
#define IDYNTREE_OPTIMALCONTROL_QUADRATICLIKECOST_H

#include <iDynTree/Cost.h>
#include <iDynTree/TimeVaryingObject.h>
#include <memory>

namespace iDynTree {
    namespace optimalcontrol {

       /**
        * @warning This class is still in active development, and so API interface can change between iDynTree versions.
        * \ingroup iDynTreeExperimental
        */

       class QuadraticLikeCost
       : public Cost {

       public:

           virtual ~QuadraticLikeCost() override;

           virtual bool costEvaluation(double time,
                                       const iDynTree::VectorDynSize& state,
                                       const iDynTree::VectorDynSize& control,
                                       double& costValue) final;//0.5 xT H x + gx + c

           virtual bool costFirstPartialDerivativeWRTState(double time,
                                                           const iDynTree::VectorDynSize& state,
                                                           const iDynTree::VectorDynSize& control,
                                                           iDynTree::VectorDynSize& partialDerivative) final;

           virtual bool costFirstPartialDerivativeWRTControl(double time,
                                                             const iDynTree::VectorDynSize& state,
                                                             const iDynTree::VectorDynSize& control,
                                                             iDynTree::VectorDynSize& partialDerivative) final;

           virtual bool costSecondPartialDerivativeWRTState(double time,
                                                            const iDynTree::VectorDynSize& state,
                                                            const iDynTree::VectorDynSize& control,
                                                            iDynTree::MatrixDynSize& partialDerivative) final;

           virtual bool costSecondPartialDerivativeWRTControl(double time,
                                                              const iDynTree::VectorDynSize& state,
                                                              const iDynTree::VectorDynSize& control,
                                                              iDynTree::MatrixDynSize& partialDerivative) final;


           virtual bool costSecondPartialDerivativeWRTStateControl(double time,
                                                                   const iDynTree::VectorDynSize& state,
                                                                   const iDynTree::VectorDynSize& control,
                                                                   iDynTree::MatrixDynSize& partialDerivative) final;

           virtual bool costSecondPartialDerivativeWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity) final;

           virtual bool costSecondPartialDerivativeWRTStateControlSparsity(iDynTree::optimalcontrol::SparsityStructure& stateControlSparsity) final;

           virtual bool costSecondPartialDerivativeWRTControlSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity) final;

       protected:
           QuadraticLikeCost(const std::string& costName);  //this class serves only as base class

           std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingMatrix> m_timeVaryingStateHessian;
           std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingVector> m_timeVaryingStateGradient;
           std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingDouble> m_timeVaryingStateCostBias;
           std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingMatrix> m_timeVaryingControlHessian;
           std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingVector> m_timeVaryingControlGradient;
           std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingDouble> m_timeVaryingControlCostBias;

           bool m_hasSecondPartialDerivativeWRTStateSparsity;
           bool m_hasSecondPartialDerivativeWRTControlSparsity;
           bool m_hasSecondPartialDerivativeWRTStateControlSparsity;

           SparsityStructure m_secondPartialDerivativeWRTStateSparsity;
           SparsityStructure m_secondPartialDerivativeWRTControlSparsity;
           SparsityStructure m_secondPartialDerivativeWRTStateControlSparsity;

       };
    }
}

#endif // IDYNTREE_OPTIMALCONTROL_QUADRATICLIKECOST_H
