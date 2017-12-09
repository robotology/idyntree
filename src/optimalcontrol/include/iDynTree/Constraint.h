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

#ifndef IDYNTREE_OPTIMALCONTROL_CONSTRAINT_H
#define IDYNTREE_OPTIMALCONTROL_CONSTRAINT_H

#include <cstddef>
#include <string>
#include "iDynTree/Core/VectorDynSize.h"

namespace iDynTree {

    class MatrixDynSize;

    namespace optimalcontrol {

    /**
     * @warning This class is still in active development, and so API interface can change between iDynTree versions.
     * \ingroup iDynTreeExperimental
     */

        class Constraint {
        public:

            Constraint(size_t size, const std::string name);

            virtual ~Constraint();

            size_t constraintSize() const;

            const std::string& name() const; //the name must not be changed

            bool setLowerBound(const VectorDynSize& lowerBound);
            bool setUpperBound(const VectorDynSize& upperBound);

            virtual bool isFeasiblePoint(double time,
                                         const VectorDynSize& state,
                                         const VectorDynSize& control) = 0;

            virtual bool evaluateConstraint(double time,
                                            const VectorDynSize& state,
                                            const VectorDynSize& control,
                                            VectorDynSize& constraint) = 0;

            virtual bool constraintJacobianWRTState(double time,
                                                    const VectorDynSize& state,
                                                    const VectorDynSize& control,
                                                    MatrixDynSize& jacobian);

            virtual bool constraintJacobianWRTControl(double time,
                                                    const VectorDynSize& state,
                                                    const VectorDynSize& control,
                                                    MatrixDynSize& jacobian);


        protected:
            size_t m_contraintSize;
            std::string m_constraintName;
            VectorDynSize m_lowerBound;
            VectorDynSize m_upperBound;
            bool m_isLowerBounded;
            bool m_isUpperBounded;
        };

    }
}

#endif /* end of include guard: IDYNTREE_OPTIMALCONTROL_CONSTRAINT_H */
