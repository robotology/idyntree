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

#ifndef IDYNTREE_OPTIMALCONTROL_CONSTRAINTSGROUP_H
#define IDYNTREE_OPTIMALCONTROL_CONSTRAINTSGROUP_H

#include <memory>
#include <string>
#include <vector>

namespace iDynTree {

    class VectorDynSize;
    class MatrixDynSize;

    namespace optimalcontrol {

        class Constraint;
        class TimeRange;

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class ConstraintsGroup{ //The group of constraint is used when there is a TimeRange associated with the constraint
        public:
            ConstraintsGroup(const std::string& name, unsigned int maxConstraintSize);

            ~ConstraintsGroup();

            ConstraintsGroup(const ConstraintsGroup& other) = delete;

            const std::string name() const;

            bool addConstraint(std::shared_ptr<Constraint> constraint, const TimeRange& timeRange);

            bool updateTimeRange(const std::string& name, const TimeRange& timeRange);

            bool removeConstraint(const std::string &name);

            bool getTimeRange(const std::string& name, TimeRange& timeRange);

            bool isFeasibilePoint(double time,
                                  const VectorDynSize& state,
                                  const VectorDynSize& control);

            bool evaluateConstraints(double time,
                                     const VectorDynSize& state,
                                     const VectorDynSize& control,
                                     VectorDynSize& constraints);

            bool getLowerBounds(double time, VectorDynSize& lowerBound);

            bool getUpperBounds(double time, VectorDynSize& upperBound);

            bool constraintJacobianWRTState(double time,
                                            const VectorDynSize& state,
                                            const VectorDynSize& control,
                                            MatrixDynSize& jacobian);

            bool constraintJacobianWRTControl(double time,
                                              const VectorDynSize& state,
                                              const VectorDynSize& control,
                                              MatrixDynSize& jacobian);

            bool isAnyTimeGroup();

            unsigned int numberOfConstraints() const;

            const std::vector<std::string> listConstraints() const;

        private:
            class ConstraintsGroupPimpl;
            ConstraintsGroupPimpl* m_pimpl;
        };
    }
}

#endif // IDYNTREE_OPTIMALCONTROL_CONSTRAINTSGROUP_H
