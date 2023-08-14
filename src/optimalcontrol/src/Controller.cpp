// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/Controller.h>

namespace iDynTree {
    namespace optimalcontrol {

    Controller::Controller(size_t controlSpaceSize)
    :m_controllerSize(controlSpaceSize)
    {}

    Controller::~Controller()
    {}

    bool Controller::setStateFeedback(double time, const VectorDynSize &stateFeedback){
        return false;
    }

    size_t Controller::controlSpaceSize(){
        return m_controllerSize;
    }

    }
}
