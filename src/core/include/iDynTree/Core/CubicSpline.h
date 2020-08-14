/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#ifndef IDYNTREE_CUBIC_SPLINE_H
#define IDYNTREE_CUBIC_SPLINE_H

#include <vector>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>

namespace iDynTree
{
    class CubicSpline {
        std::vector< iDynTree::Vector4 > m_coefficients;
        iDynTree::VectorDynSize m_velocities;
        iDynTree::VectorDynSize m_time;
        iDynTree::VectorDynSize m_y;
        iDynTree::VectorDynSize m_T;

        double m_v0;
        double m_vf;
        double m_a0;
        double m_af;

        bool computePhasesDuration();
        bool computeIntermediateVelocities();
        bool computeCoefficients();

        bool m_areCoefficientsUpdated;

    public:
        CubicSpline();

        CubicSpline(unsigned int buffersDimension);

        bool setData(const iDynTree::VectorDynSize& time, const iDynTree::VectorDynSize& yData);
        void setInitialConditions(double initialVelocity, double initialAcceleration);
        void setFinalConditions(double finalVelocity, double finalAcceleration);
        double evaluatePoint(double t);
        double evaluatePoint(double t, double& velocity, double& acceleration);
    };
}

#endif
