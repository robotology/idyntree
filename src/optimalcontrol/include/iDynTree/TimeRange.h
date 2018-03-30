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

#ifndef IDYNTREE_OPTIMALCONTROL_TIMERANGE_H
#define IDYNTREE_OPTIMALCONTROL_TIMERANGE_H

namespace iDynTree{
    namespace optimalcontrol{

        /**
         * @warning This class is still in active development, and so API interface can change between iDynTree versions.
         * \ingroup iDynTreeExperimental
         */

        class TimeRange {
            double m_initTime;
            double m_endTime;
            bool m_anyTime;
        public:
            TimeRange();
            TimeRange(const double init, const double end);

            double initTime() const;
            double endTime() const;
            double length() const;
            bool setTimeInterval(const double init, const double end);
            bool operator<(const TimeRange &rhs) const; //The comparison is only on the init time
            bool operator==(const TimeRange &rhs) const;
            bool operator!=(const TimeRange &rhs) const;
            bool isValid() const;
            bool isInRange(double time) const;
            static TimeRange AnyTime();
            static TimeRange Instant(const double time);
        };
    }
}

#endif // IDYNTREE_OPTIMALCONTROL_TIMERANGE_H
