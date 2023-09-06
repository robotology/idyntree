// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/TimeRange.h>
#include <iDynTree/Utils.h>

namespace iDynTree {
    namespace optimalcontrol {
        TimeRange::TimeRange()
        :m_initTime(0.0)
        ,m_endTime(0.0)
        ,m_anyTime(false)
        {
        }

        TimeRange::TimeRange(const double init, const double end)
        :m_anyTime(false)
        {
            if(!setTimeInterval(init, end)){
                reportError("TimeRange", "TimeRange", "Invalid initialization. Setting equal to AnyTime.");
                m_initTime = -1;
                m_endTime = -1;
                m_anyTime = true;
            }
        }

        double TimeRange::initTime() const
        {
            return m_initTime;
        }

        double TimeRange::endTime() const
        {
            return m_endTime;
        }

        double TimeRange::length() const
        {
            return m_endTime - m_initTime;
        }


        bool TimeRange::setTimeInterval(const double init, const double end)
        {
            if(init > end){
                reportError("TimeRange", "setTimeInterval", "The init time should be grater than the end.");
                return false;
            }

            m_initTime = init;
            m_endTime = end;
            m_anyTime = false;

            return true;
        }

        TimeRange TimeRange::AnyTime()
        {
            TimeRange output;
            output.m_initTime = -1;
            output.m_endTime = -1;
            output.m_anyTime = true;
            return output;
        }

        TimeRange TimeRange::Instant(const double time)
        {
            return TimeRange(time, time);
        }

        bool TimeRange::operator<(const TimeRange &rhs) const
        {
            if (this->m_anyTime && rhs.m_anyTime) {
                return false;
            }

            if (rhs.m_anyTime) {
                return true;
            }

            if (this ->m_anyTime) {
                return false;
            }

            if(this->m_initTime != rhs.initTime()) {
                return this->m_initTime < rhs.initTime();
            } else {
                return this->m_endTime < rhs.endTime();
            }
        }

        bool TimeRange::operator==(const TimeRange &rhs) const
        {
            if (this->m_anyTime && rhs.m_anyTime) {
                return true;
            }

            if (rhs.m_anyTime) {
                return false;
            }

            if (this ->m_anyTime) {
                return false;
            }

            return (checkDoublesAreEqual(this->m_initTime, rhs.initTime()) && checkDoublesAreEqual(this->m_endTime, rhs.endTime()));//((this->m_initTime == rhs.initTime())&&(this->m_endTime == rhs.endTime()));
        }

        bool TimeRange::operator!=(const TimeRange &rhs) const
        {
            return !(this->operator==(rhs));
        }

        bool TimeRange::isValid() const
        {
            return (m_initTime <= m_endTime);
        }

        bool TimeRange::isInRange(double time) const
        {
            if (m_anyTime) {
                return true;
            }

            if (isInstant()) {
                return checkDoublesAreEqual(m_initTime, time);
            }

            return ((m_initTime <= time) && (m_endTime >= time));
        }

        bool TimeRange::isInstant() const
        {
            return (!m_anyTime) && checkDoublesAreEqual(m_initTime, m_endTime);
        }

    }

}
