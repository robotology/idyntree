// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <iDynTree/Utils.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/SO3Utils.h>
#include <iDynTree/EigenHelpers.h>
#include <Eigen/Dense>

#include <cmath>
#include <numeric>

void checkIsValidRotationMatrix()
{
    ASSERT_IS_TRUE(iDynTree::isValidRotationMatrix(iDynTree::Rotation::Identity()));
    ASSERT_IS_TRUE(iDynTree::isValidRotationMatrix(iDynTree::getRandomRotation()));
    iDynTree::Rotation test;
    iDynTree::getRandomMatrix(test);
    ASSERT_IS_TRUE(!iDynTree::isValidRotationMatrix(test));
}

void checkGeodesicDistance()
{
    iDynTree::Rotation r1 = iDynTree::getRandomRotation();
    iDynTree::Rotation r2 = iDynTree::getRandomRotation();

    iDynTree::Rotation error = r1.inverse()*r2;

    Eigen::AngleAxisd aa(iDynTree::toEigen(error));

    double distance = iDynTree::geodesicL2Distance(r1, r2);

    ASSERT_EQUAL_DOUBLE(std::abs(aa.angle()), distance);

    r1 = iDynTree::Rotation::Identity();
    r2 = iDynTree::Rotation::RPY(0.0, 0.0, 0.5);

    distance = iDynTree::geodesicL2Distance(r1, r2);

    ASSERT_EQUAL_DOUBLE(0.5, distance);

}

void checkWeightedMeanRotation()
{
    size_t numberOfRotations = 100;
    double perturbation = M_PI * 0.8;
    std::vector<iDynTree::Rotation> rotations;
    std::vector<double> weights;
    double originalRoll, originalPitch, originalYaw;

    rotations.push_back(iDynTree::getRandomRotation());
    rotations.back().getRPY(originalRoll, originalPitch, originalYaw);
    weights.push_back(iDynTree::getRandomDouble());

    for (size_t i = 1; i < numberOfRotations; ++i)
    {
        rotations.push_back(iDynTree::Rotation::RPY(originalRoll + iDynTree::getRandomDouble(-0.5 * perturbation, 0.5 * perturbation),
                                                    originalPitch + iDynTree::getRandomDouble(-0.5 * perturbation, 0.5 * perturbation),
                                                    originalYaw + iDynTree::getRandomDouble(-0.5 * perturbation, 0.5 * perturbation)));
        weights.push_back(iDynTree::getRandomDouble());
    }
    double totalWeight = std::accumulate(weights.begin(), weights.end(), 0.0);

    iDynTree::Rotation weightedMean;

    iDynTree::GeodesicL2MeanOptions options;
    options.verbose = true;
    options.maxIterations = 1000;
    options.stepSize = 1.0;

    bool ok = iDynTree::geodesicL2WeightedMeanRotation(rotations, weights, weightedMean, options);

    ASSERT_IS_TRUE(ok);
    ASSERT_IS_TRUE(iDynTree::isValidRotationMatrix(weightedMean));

    Eigen::Vector3d r;
    r.setZero();

    for (size_t idx = 0; idx < numberOfRotations; ++idx)
    {
        const iDynTree::Rotation& R_i = rotations[idx];
        double w_i = weights[idx];
        r =  r + w_i * toEigen((weightedMean.inverse() * R_i).log());
    }
    r = r * (1.0 / totalWeight);

    ASSERT_IS_TRUE(r.norm() < options.tolerance);

}

void checkMeanRotation()
{
    size_t numberOfRotations = 100;
    double perturbation = M_PI * 0.8;
    std::vector<iDynTree::Rotation> rotations;
    double originalRoll, originalPitch, originalYaw;

    rotations.push_back(iDynTree::getRandomRotation());
    rotations.back().getRPY(originalRoll, originalPitch, originalYaw);

    for (size_t i = 1; i < numberOfRotations; ++i)
    {
        rotations.push_back(iDynTree::Rotation::RPY(originalRoll + iDynTree::getRandomDouble(-0.5 * perturbation, 0.5 * perturbation),
                                                    originalPitch + iDynTree::getRandomDouble(-0.5 * perturbation, 0.5 * perturbation),
                                                    originalYaw + iDynTree::getRandomDouble(-0.5 * perturbation, 0.5 * perturbation)));
    }

    iDynTree::Rotation weightedMean;

    iDynTree::GeodesicL2MeanOptions options;
    options.verbose = true;
    options.maxIterations = 1000;

    bool ok = iDynTree::geodesicL2MeanRotation(rotations, weightedMean, options);
    ASSERT_IS_TRUE(ok);
    ASSERT_IS_TRUE(iDynTree::isValidRotationMatrix(weightedMean));


    Eigen::Vector3d r;
    r.setZero();

    for (size_t idx = 0; idx < numberOfRotations; ++idx)
    {
        const iDynTree::Rotation& R_i = rotations[idx];
        r =  r + toEigen((weightedMean.inverse() * R_i).log());
    }
    r = r * (1.0 / numberOfRotations);

    ASSERT_IS_TRUE(r.norm() < options.tolerance);

}

int main()
{

    checkIsValidRotationMatrix();
    checkGeodesicDistance();
    checkWeightedMeanRotation();
    checkMeanRotation();

    return EXIT_SUCCESS;
}
