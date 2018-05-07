/*
 * Copyright (C) 2014,2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2017.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_LINEARSYSTEM_H
#define IDYNTREE_OPTIMALCONTROL_LINEARSYSTEM_H

#include "DynamicalSystem.h"

#include <vector>
namespace iDynTree {

    class MatrixDynSize;

    namespace optimalcontrol {

        class Controller;
        // Add somewhere the idea of output. In case of time varying system it may share the same parameter of
        // the A or B matrix

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class LinearSystem 
        : public iDynTree::optimalcontrol::DynamicalSystem {

        public:

            LinearSystem(size_t stateSize,
                         size_t controlSize,
                         bool isTimeVarying);

            LinearSystem(const LinearSystem& other) = delete;

            ~LinearSystem();

            bool isTimeVarying();

            // time invariant system: single matrix
            void setConstantStateMatrix(const iDynTree::MatrixDynSize&);
            void setConstantControlMatrix(const iDynTree::MatrixDynSize&);

            // time varying cases:
            // 1) preallocate matrices [initialTime, finalTime)
            void setStateMatrixForTimeRange(const iDynTree::MatrixDynSize&, double initialTime, double finalTime);
            //or
            void setStateMatricesForTimeRanges(std::vector<const iDynTree::MatrixDynSize&>,
                                               std::vector<std::pair<double, double> >);

            // 2) lambda which will be called at runtime (how this will work with eventual derivatives?)
            // Is this feasible?
            // Ideally I would call the lambda as  (sender, time)-> iDynTree::MatrixDynSize&
            // But if the lambda captures variable it cannot be passed as function pointer.
            // Alternatevely, I can do a C-style call, like registering a function pointer of time
            // iDynTree::MatrixDynSize& (*stateCallback)(const LinearSystem& sender, double time, void* context)
            // context can be anything, only the caller knows about it.

            iDynTree::MatrixDynSize& stateMatrix(double time) const;
            iDynTree::MatrixDynSize& controlMatrix(double time) const;

            virtual bool dynamics(const VectorDynSize& state,
                                  double time,
                                  VectorDynSize& stateDynamics) override;

            virtual bool dynamicsStateFirstDerivative(const VectorDynSize& state,
                                                      double time,
                                                      MatrixDynSize& dynamicsDerivative) override;

            virtual bool dynamicsControlFirstDerivative(const VectorDynSize& state,
                                                        double time,
                                                        MatrixDynSize& dynamicsDerivative) override;


        private:
            class LinearSystemPimpl;
            LinearSystemPimpl* m_pimpl;

        };
    }
}


#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_LINEARSYSTEM_H */
