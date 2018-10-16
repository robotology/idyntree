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


#ifndef IDYNTREE_OPTIMALCONTROL_TIMEVARYINGOBJECT_H
#define IDYNTREE_OPTIMALCONTROL_TIMEVARYINGOBJECT_H

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>

namespace iDynTree {

    namespace optimalcontrol {

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */
        template<typename Object>
        class TimeVaryingObject {
        public:
            TimeVaryingObject(){}

            virtual ~TimeVaryingObject() {}

            virtual const Object& get(double time, bool &isValid) = 0;
        };

        typedef TimeVaryingObject<VectorDynSize> TimeVaryingVector;

        typedef TimeVaryingObject<MatrixDynSize> TimeVaryingMatrix;

        typedef TimeVaryingObject<double> TimeVaryingDouble;

        typedef TimeVaryingObject<Transform> TimeVaryingTransform;

        typedef TimeVaryingObject<Rotation> TimeVaryingRotation;

        typedef TimeVaryingObject<Position> TimeVaryingPosition;


        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        template <typename Object>
        class TimeInvariantObject : public TimeVaryingObject<Object> {
            Object m_invariantObject;
        public:
            TimeInvariantObject();

            TimeInvariantObject(const Object& timeInvariantObject);

            ~TimeInvariantObject() final;

            Object& get();

            virtual const Object& get(double time, bool &isValid) final;
        };

        extern template class TimeInvariantObject<double>;

        extern template class TimeInvariantObject<VectorDynSize>;

        extern template class TimeInvariantObject<MatrixDynSize>;

        extern template class TimeInvariantObject<Transform>;

        extern template class TimeInvariantObject<Rotation>;

        extern template class TimeInvariantObject<Position>;

        typedef TimeInvariantObject<double> TimeInvariantDouble;

        typedef TimeInvariantObject<VectorDynSize> TimeInvariantVector;

        typedef TimeInvariantObject<MatrixDynSize> TimeInvariantMatrix;

        typedef TimeInvariantObject<Transform> TimeInvariantTransform;

        typedef TimeInvariantObject<Rotation> TimeInvariantRotation;

        typedef TimeInvariantObject<Position> TimeInvariantPosition;
    }
}

#endif // IDYNTREE_OPTIMALCONTROL_TIMEVARYINGOBJECT_H
