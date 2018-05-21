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
