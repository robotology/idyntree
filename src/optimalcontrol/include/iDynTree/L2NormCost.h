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


#ifndef IDYNTREE_OPTIMALCONTROL_L2NORMCOST_H
#define IDYNTREE_OPTIMALCONTROL_L2NORMCOST_H

#include <iDynTree/QuadraticCost.h>
#include <iDynTree/TimeVaryingObject.h>
#include <string>

namespace iDynTree {

    class MatrixDynSize;
    class VectorDynSize;

    namespace optimalcontrol {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class L2NormCost : public QuadraticCost {
            class L2NormCostImplementation;
            L2NormCostImplementation *m_pimpl;

            using QuadraticCost::setStateCost;
            using QuadraticCost::setControlCost;

        public:
            L2NormCost(const std::string& name);

            L2NormCost(const std::string& name, const MatrixDynSize& stateSelector); //stateSelector premultiplies the state in the L2-norm

            L2NormCost(const std::string& name, const MatrixDynSize& stateSelector, const MatrixDynSize& controlSelector); //controlSelector premultiplies the control in the L2-norm

            ~L2NormCost();

            bool setStateWeight(const VectorDynSize& stateWeights);

            bool setStateWeight(const MatrixDynSize& stateWeights);

            bool setControlWeight(const VectorDynSize& controlWeights);

            bool setControlWeight(const MatrixDynSize& controlWeights);
        };

    }
}

#endif // IDYNTREE_OPTIMALCONTROL_L2NORMCOST_H
