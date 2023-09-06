// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <iDynTree/SO3Utils.h>
#include <iDynTree/Utils.h>
#include <iDynTree/EigenHelpers.h>

#include <cmath>
#include <chrono>
#include <string>
#include <numeric>

bool iDynTree::isValidRotationMatrix(const iDynTree::Rotation &r)
{
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> map = iDynTree::toEigen(r);
    return std::isfinite(map.norm())
        && iDynTree::checkDoublesAreEqual(map.determinant(), 1.0)
        && (map * map.transpose()).isApprox(iDynTree::toEigen(iDynTree::Rotation::Identity()));
}

double iDynTree::geodesicL2Distance(const iDynTree::Rotation& rotation1, const iDynTree::Rotation& rotation2)
{
    Eigen::Vector3d angvelEigen = iDynTree::toEigen((rotation1.inverse()*rotation2).log());

    return angvelEigen.norm();
}

bool iDynTree::geodesicL2MeanRotation(const std::vector<iDynTree::Rotation>& inputRotations,
                                      iDynTree::Rotation& meanRotation,
                                      const GeodesicL2MeanOptions& options)

{
    return geodesicL2WeightedMeanRotation(inputRotations, std::vector<double>(), meanRotation, options);
}

bool iDynTree::geodesicL2WeightedMeanRotation(const std::vector<iDynTree::Rotation>& inputRotations,
                                              const std::vector<double>& weights,
                                              iDynTree::Rotation& meanRotation,
                                              const GeodesicL2MeanOptions& options)
{
    using iDynTree::toEigen;

    if (!inputRotations.size())
    {
        iDynTree::reportError("SO3Utils", "geodesicL2WeightedMeanRotation", "Empty inputRotations vector.");
        return false;
    }

    if (weights.size() && (inputRotations.size() != weights.size()))
    {
        iDynTree::reportError("SO3Utils", "geodesicL2WeightedMeanRotation", "Vectors size mismatch: weights and inputRotations must be same size.");
        return false;
    }

    if (options.tolerance < 0.0)
    {
        iDynTree::reportError("SO3Utils", "geodesicL2WeightedMeanRotation", "The tolerance is supposed to be a positive number.");
        return false;
    }

    if (options.stepSize < 0.0)
    {
        iDynTree::reportError("SO3Utils", "geodesicL2WeightedMeanRotation", "The stepSize is supposed to be non-negative.");
        return false;
    }

    double nrRot = inputRotations.size();

    double totalWeight = 0.0;
    if (weights.size())
    {
        for (size_t i = 0; i < weights.size(); ++i)
        {
            if (weights[i] < 0.0)
            {
                std::stringstream ss;
                ss << "The weight at index " << i << " is negative.";
                iDynTree::reportError("SO3Utils", "geodesicL2WeightedMeanRotation", ss.str().c_str());
                return false;
            }
            totalWeight += weights[i];
        }
    }
    else
    {
        totalWeight = nrRot;
    }

    if (totalWeight < options.tolerance)
    {
        iDynTree::reportError("SO3Utils", "geodesicL2WeightedMeanRotation", "The sum of all the weights is lower than the tolerance.");
        return false;
    }

    // initial condition for optimization
    meanRotation = inputRotations[0];

    bool optimal_R_found{false};
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    int iteration = 1;
    while (!optimal_R_found)
    {
        // Riemannian gradient descent with constant step size
        iDynTree::AngularMotionVector3 perturbation;
        perturbation.zero();
        Eigen::Map<Eigen::Vector3d> r(toEigen(perturbation));

        for (size_t idx = 0; idx < nrRot; ++idx)
        {
            const iDynTree::Rotation& R_i = inputRotations[idx];
            double w_i = weights.size() ? weights[idx] : 1.0;

            r =  r + w_i * toEigen((meanRotation.inverse() * R_i).log());
        }
        r = r * (1.0 / totalWeight);

        double norm = r.norm();

        if (std::isinf(norm))
        {
            iDynTree::reportError("SO3Utils", "geodesicL2WeightedMeanRotation", "Inf detected.");
            return false;
        }

        if (std::isnan(norm))
        {
            iDynTree::reportError("SO3Utils", "geodesicL2WeightedMeanRotation", "Nan detected.");
            return false;
        }

        if (r.norm() <= options.tolerance)
        {
            optimal_R_found = true;
            continue;
        }


        r *= options.stepSize;

        meanRotation = meanRotation * perturbation.exp();

        if (!isValidRotationMatrix(meanRotation))
        {
            iDynTree::reportError("SO3Utils", "geodesicL2WeightedMeanRotation", "The computed mean rotation is no more a rotation matrix.");
            return false;
        }

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        std::chrono::duration<double> timout_check = now - start;
        double time_lapsed{timout_check.count()};

        if ((options.timeoutInSeconds > 0.0) && (time_lapsed > options.timeoutInSeconds))
        {
            iDynTree::reportError("SO3Utils", "geodesicL2WeightedMeanRotation", "Timeout reached, optimal mean not found.");
            return false;
        }

        if ((options.maxIterations > 0) && (iteration >= options.maxIterations))
        {
            iDynTree::reportError("SO3Utils", "geodesicL2WeightedMeanRotation", "Maximum iteration reached, optimal mean not found.");
            return false;
        }

        iteration++;

    }
    std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();

    std::chrono::duration<double> elapsed = finish - start;
    std::stringstream ss;
    ss << "Optimization took " << elapsed.count() << " seconds in " << iteration << " iterations.";

    if (options.verbose)
    {
        iDynTree::reportInfo("SO3Utils", "geodesicL2WeightedMeanRotation", ss.str().c_str());
    }

    return true;
}

