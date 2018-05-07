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

#ifndef IDYNTREE_OPTIMALCONTROL_DYNAMICALSYSTEM_H
#define IDYNTREE_OPTIMALCONTROL_DYNAMICALSYSTEM_H

#include <iDynTree/Core/VectorDynSize.h>

#include <cstddef>
//#include <memory>

namespace iDynTree {

    class MatrixDynSize;

namespace optimalcontrol {

    /**
     * @warning This class is still in active development, and so API interface can change between iDynTree versions.
     * \ingroup iDynTreeExperimental
     */

    class DynamicalSystem {

    public:

        DynamicalSystem() = delete;

        // TODO: add also constant parameters
        DynamicalSystem(size_t stateSpaceSize,
                        size_t controlSpaceSize);

        DynamicalSystem(const DynamicalSystem& other) = delete;

        virtual ~DynamicalSystem();

        size_t stateSpaceSize() const;
        size_t controlSpaceSize() const;

        virtual bool dynamics(const VectorDynSize& state,
                              double time,
                              VectorDynSize& stateDynamics) = 0;

        virtual bool setControlInput(const VectorDynSize &control);

        virtual const VectorDynSize& controlInput() const;

        virtual double controlInput(unsigned int index) const;

        virtual const VectorDynSize& initialState() const;

        virtual double initialState(unsigned int index) const;

        virtual bool setInitialState(const VectorDynSize &state);

        //TODO: add also second derivative?
        virtual bool dynamicsStateFirstDerivative(const VectorDynSize& state,
                                                  double time,
                                                  MatrixDynSize& dynamicsDerivative);

        virtual bool dynamicsControlFirstDerivative(const VectorDynSize& state,
                                                    double time,
                                                    MatrixDynSize& dynamicsDerivative);

    private:
        // TODO: does it make sense to have members in this abstrac class?
        size_t m_stateSize;
        size_t m_controlSize;
        VectorDynSize m_initialState;
        VectorDynSize m_controlInput;
    };
    
}
}



#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_DYNAMICALSYSTEM_H */
