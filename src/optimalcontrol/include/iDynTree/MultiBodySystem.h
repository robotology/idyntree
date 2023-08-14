// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_MULTIBODYSYSTEM_H
#define IDYNTREE_OPTIMALCONTROL_MULTIBODYSYSTEM_H

#include <iDynTree/DynamicalSystem.h>

namespace iDynTree {

    class Model;

    namespace optimalcontrol {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class MultiBodySystem 
        : public iDynTree::optimalcontrol::DynamicalSystem {

        public:

            MultiBodySystem(const iDynTree::Model& );

        };
    }
}


#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_MULTIBODYSYSTEM_H */
