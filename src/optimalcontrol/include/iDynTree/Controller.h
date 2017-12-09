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

    class Controller{
        size_t m_controllerSize;
    public:
        Controller(size_t controlSpaceSize);

        virtual ~Controller();

        virtual bool doControl(VectorDynSize& controllerOutput) = 0;

        virtual bool setStateFeedback(const double t, const VectorDynSize& stateFeedback);

        size_t controlSpaceSize();
    };
}
}


#endif // IDYNTREE_OPTIMALCONTROL_CONTROLLER_H
