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
        size_t m_stateSize;
        size_t m_controlSize;
        VectorDynSize m_initialState;
        VectorDynSize m_controlInput;
    };
    
}
}



#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_DYNAMICALSYSTEM_H */
