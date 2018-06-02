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

            virtual const Object& getObject(double time, bool &isValid) = 0;

        };

        typedef TimeVaryingObject<VectorDynSize> TimeVaryingVector;

        typedef TimeVaryingObject<MatrixDynSize> TimeVaryingMatrix;

        typedef TimeVaryingObject<double> TimeVaryingDouble;

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */
        class TimeInvariantVector : public TimeVaryingVector {
            VectorDynSize m_timeInvariantVector;
        public:
            TimeInvariantVector();

            TimeInvariantVector(const VectorDynSize& timeInvariantVector);

            VectorDynSize& get();

            virtual const VectorDynSize& getObject(double time, bool &isValid) override;
        };

        class TimeInvariantMatrix : public TimeVaryingMatrix {
            MatrixDynSize m_timeInvariantMatrix;
        public:
            TimeInvariantMatrix();

            TimeInvariantMatrix(const MatrixDynSize& timeInvariantMatrix);

            MatrixDynSize& get();

            virtual const MatrixDynSize& getObject(double time, bool &isValid) override;
        };

        class TimeInvariantDouble : public TimeVaryingDouble {
            double m_timeInvariantDouble;
        public:
            TimeInvariantDouble();

            TimeInvariantDouble(double timeInvariantDouble);

            double& get();

            virtual const double& getObject(double time, bool &isValid) override;
        };

    }
}

#endif // IDYNTREE_OPTIMALCONTROL_TIMEVARYINGOBJECT_H
