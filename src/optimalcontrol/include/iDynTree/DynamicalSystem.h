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

#include <cstddef>
//#include <memory>

namespace iDynTree {

    class VectorDynSize;
    class MatrixDynSize;

namespace optimalcontrol {
    class Controller;

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
                              VectorDynSize& stateDynamics);

        virtual bool dynamics(const VectorDynSize &state, const VectorDynSize &control,
                              double time, VectorDynSize &stateDynamics);

        virtual const VectorDynSize& initialState() const = 0;

        //TODO: for now putting these functions as not virtual
        //TODO: add also second derivative?
        virtual bool dynamicsStateFirstDerivative(const VectorDynSize& state,
                                                  const VectorDynSize& control,
                                                  double time,
                                                  MatrixDynSize& dynamicsDerivative);

        virtual bool dynamicsControlFirstDerivative(const VectorDynSize& state,
                                                    const VectorDynSize& control,
                                                    double time,
                                                    MatrixDynSize& dynamicsDerivative);

//        bool setController(std::shared_ptr<Controller> controllerPointer); //For the time being we have difficulties to abstract the controller. This avoid the usage of casts.

//        const std::weak_ptr<const Controller> controller() const;



    protected:
        // TODO: does it make sense to have members in this abstrac class?
        size_t m_stateSize;
        size_t m_controlSize;
//        std::shared_ptr<Controller> m_controller_ptr;
    };
    
}
}



#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_DYNAMICALSYSTEM_H */
