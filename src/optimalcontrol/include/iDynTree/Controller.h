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

#ifndef IDYNTREE_OPTIMALCONTROL_CONTROLLER_H
#define IDYNTREE_OPTIMALCONTROL_CONTROLLER_H

#include <cstddef>

namespace iDynTree {

    class VectorDynSize;

    namespace optimalcontrol{

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        /**
         * @brief Class describing a generic controller.
         */
        class Controller{
            size_t m_controllerSize;
        public:
            /**
             * @brief Controller's constructor
             * @param[in] controlSpaceSize The control space dimension.
             */
            Controller(size_t controlSpaceSize);

            virtual ~Controller();

            /**
             * @brief Evaluate the control action
             * @param[out] controllerOutput The output of the controller.
             * @return True if successfull.
             */
            virtual bool doControl(VectorDynSize& controllerOutput) = 0;

            /**
             * @brief Set the state feedback to be used inside the controller.
             * If the controller depends upon some outputs of the dynamical system, those outputs should be computed inside this method using the state and time specified.
             * @param[in] time The time in which the feedback is obtained.
             * @param[in] stateFeedback The state feedback
             * @return True if successfull. In this specific class, this methods always outputs false.
             */
            virtual bool setStateFeedback(double time, const VectorDynSize& stateFeedback);

            /**
             * @brief The dimension of control space.
             * @return The dimension of control space.
             */
            size_t controlSpaceSize();
        };
    }
}


#endif // IDYNTREE_OPTIMALCONTROL_CONTROLLER_H
