/*
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#ifndef IDYNTREE_SO3UTILS_H
#define IDYNTREE_SO3UTILS_H

#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/AngularMotionVector3.h>
#include <vector>

namespace iDynTree
{

    struct GeodesicL2MeanOptions
    {
        double tolerance{1e-5};
        double timeoutInSeconds{-1};
        int maxIterations{-1};
        bool verbose{false};
        double stepSize{1.0};
    };

    double geodesicL2Distance(const iDynTree::Rotation& rotation1, const iDynTree::Rotation& rotation2);


    bool geodesicL2MeanRotation(const std::vector<iDynTree::Rotation>& inputRotations,
                                iDynTree::Rotation& meanRotation,
                                const GeodesicL2MeanOptions& options = GeodesicL2MeanOptions());


    bool geodesicL2WeightedMeanRotation(const std::vector<iDynTree::Rotation>& inputRotations,
                                         const std::vector<double>& weights,
                                         iDynTree::Rotation& meanRotation,
                                         const GeodesicL2MeanOptions& options = GeodesicL2MeanOptions());
}

#endif // IDYNTREE_SO3UTILS_H
