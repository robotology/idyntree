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
#include <iDynTree/Core/GeomVector3.h>
#include <vector>

namespace iDynTree
{
    /**
     * @brief Struct containing the options for geodesicL2MeanRotation and geodesicL2WeightedMeanRotation.
     */
    struct GeodesicL2MeanOptions
    {
        double tolerance{1e-5}; /** Tolerance for terminating the inner refinement loop of the mean rotation. **/
        double timeoutInSeconds{-1}; /** Timeout for the refinement loop. **/
        int maxIterations{-1}; /** Max number of iterations for the refinement loop. **/
        bool verbose{false}; /** Add a message when the solution is found. **/
        double stepSize{1.0}; /** Step-size for the refinement loop. **/
    };

    /**
     * @brief Check if the rotation matrix is valid.
     *
     * It checks that the determinant is 1, that the Frobenius norm is finite and that it is orthogonal.
     * @param r The input rotation
     * @return True if it is a rotation matrix.
     */
    bool isValidRotationMatrix(const iDynTree::Rotation& r);

    /**
     * @brief Computes the geodesic distance between two rotation matrices.
     *
     * It implements the angular distance presented in Sec. 4 of "Rotation Averaging" (available at http://users.cecs.anu.edu.au/~hongdong/rotationaveraging.pdf),
     * in particular \f$d = ||log(R_1^\top R_2)||^2 \f$.
     * @param rotation1 The first rotation.
     * @param rotation2 The other rotation.
     * @return the geodesic L2 distance between the two rotation matrices.
     */
    double geodesicL2Distance(const iDynTree::Rotation& rotation1, const iDynTree::Rotation& rotation2);

    /**
     * @brief Computes the geodesic mean amongst the provided rotations.
     *
     * It implements Algorithm 1 in Sec. 5.3 of "Rotation Averaging" (available at http://users.cecs.anu.edu.au/~hongdong/rotationaveraging.pdf).
     *
     * Inside it calls geodesicL2WeightedMeanRotation.
     *
     * @param inputRotations The rotations to average.
     * @param meanRotation The mean rotation.
     * @param options The options for the inner optimization (refinement) loop.
     * @return false in case of failure, true otherwise.
     */
    bool geodesicL2MeanRotation(const std::vector<iDynTree::Rotation>& inputRotations,
                                iDynTree::Rotation& meanRotation,
                                const GeodesicL2MeanOptions& options = GeodesicL2MeanOptions());

    /**
     * @brief Computes the weighted geodesic mean amongst the provided rotations
     *
     * It implements Algorithm 1 in Sec. 5.3 of "Rotation Averaging" (available at http://users.cecs.anu.edu.au/~hongdong/rotationaveraging.pdf),
     * with a small modification to take into accounts weights different from 1.
     *
     * @param inputRotations The rotations to average.
     * @param weights The weights for each rotation. If this vector is null assumes that each weight is 1.0 (equivalent to geodesicL2MeanRotation)
     * @param meanRotation The weighted mean rotation.
     * @param options The options for the inner optimization (refinement) loop.
     * @return false in case of failure, true otherwise.
     */
    bool geodesicL2WeightedMeanRotation(const std::vector<iDynTree::Rotation>& inputRotations,
                                         const std::vector<double>& weights,
                                         iDynTree::Rotation& meanRotation,
                                         const GeodesicL2MeanOptions& options = GeodesicL2MeanOptions());
}

#endif // IDYNTREE_SO3UTILS_H
