// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#ifndef IDYNTREE_CUBIC_SPLINE_H
#define IDYNTREE_CUBIC_SPLINE_H

#include <vector>
#include <iDynTree/VectorFixSize.h>
#include <iDynTree/VectorDynSize.h>

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
