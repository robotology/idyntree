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
