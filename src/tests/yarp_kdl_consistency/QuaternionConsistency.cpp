/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/TestUtils.h>

#include <kdl_codyco/KDLConversions.h>
#include <kdl/frames.hpp>

#include <iDynTree/yarp/YARPConversions.h>

#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <Eigen/Geometry>

#include <cstdlib>
#include <limits>
#include <iostream>
#include <sstream>


class MaxMin
{
public:
    double min;
    double max;

    MaxMin()
    {
        min = std::numeric_limits<double>::max();
        max = -std::numeric_limits<double>::max();
    }

    void update(const double newValue)
    {
        if( newValue > max )
        {
            max = newValue;
        }

        if( newValue < min )
        {
            min = newValue;
        }
    }
};

class QuaternionMaxMin
{
public:
    MaxMin help[4];

    void update(const iDynTree::Vector4 & quat)
    {
        help[0].update(quat(0));
        help[1].update(quat(1));
        help[2].update(quat(2));
        help[3].update(quat(3));
    }

    std::string toString()
    {
        std::stringstream ss;
        ss << "Real part is in (" << (help[0].min) << " , " << (help[0].max) << ")" << std::endl;
        ss << "Imag x    is in (" << (help[1].min) << " , " << (help[1].max) << ")" << std::endl;
        ss << "Imag y    is in (" << (help[2].min) << " , " << (help[2].max) << ")" << std::endl;
        ss << "Imag z    is in (" << (help[3].min) << " , " << (help[3].max) << ")" << std::endl;

        return ss.str();
    }
};

struct QuaternionMaxMinHelper
{
    QuaternionMaxMin idyntree;
    QuaternionMaxMin yarp;
    QuaternionMaxMin kdl;
    QuaternionMaxMin eigen;
};

void getQuaternion(const iDynTree::Rotation & rot,
                   iDynTree::Vector4 & quat_idyntree,
                   iDynTree::Vector4 & quat_kdl,
                   iDynTree::Vector4 & quat_yarp,
                   iDynTree::Vector4 & quat_eigen,
                   iDynTree::Rotation & rotBack_idyntree,
                   iDynTree::Rotation & rotBack_kdl,
                   iDynTree::Rotation & rotBack_yarp,
                   iDynTree::Rotation & rotBack_eigen)
{
    rot.getQuaternion(quat_idyntree);

    // Notice that KDL uses a imaginary/real serialization, while we use real/imaginary
    iDynTree::ToKDL(rot).GetQuaternion(quat_kdl(1),quat_kdl(2),quat_kdl(3),quat_kdl(0));

    yarp::sig::Matrix rotYarp(3,3);
    iDynTree::toYarp(rot,rotYarp);

    yarp::sig::Vector quat_yarp_yarp(4);

    quat_yarp_yarp = yarp::math::dcm2quat(rotYarp);

    iDynTree::toiDynTree(quat_yarp_yarp,quat_yarp);

    Eigen::Quaternion<double> quat_eigen_eigen = Eigen::Quaternion<double>(iDynTree::toEigen(rot));

    // Eigen internally stores quaternions as imaginary/real
    quat_eigen(0) = quat_eigen_eigen.w();
    quat_eigen(1) = quat_eigen_eigen.x();
    quat_eigen(2) = quat_eigen_eigen.y();
    quat_eigen(3) = quat_eigen_eigen.z();

    // We convert the quaternion back to rotations always using the iDynTree function, to make sure that all quaternions are compatible
    rotBack_idyntree = iDynTree::Rotation::RotationFromQuaternion(quat_idyntree);
    rotBack_kdl = iDynTree::Rotation::RotationFromQuaternion(quat_kdl);
    rotBack_yarp = iDynTree::Rotation::RotationFromQuaternion(quat_yarp);
    rotBack_eigen = iDynTree::Rotation::RotationFromQuaternion(quat_eigen);

}

void checkRotationToQuaternion(const iDynTree::Rotation randRot, QuaternionMaxMinHelper & helper, bool verbose)
{
    iDynTree::Vector4 quat_idyntree, quat_kdl, quat_yarp, quat_eigen;
    iDynTree::Rotation rotBack_idyntree, rotBack_kdl, rotBack_yarp, rotBack_eigen;
    getQuaternion(randRot,quat_idyntree,quat_kdl,quat_yarp,quat_eigen,rotBack_idyntree,rotBack_kdl,rotBack_yarp,rotBack_eigen);

    if( verbose )
    {
        std::cerr << "Original rotation:\n" << randRot.toString() << std::endl;
        std::cerr << "Quaternion converted by iDynTree: " << quat_idyntree.toString() << " ( norm " << toEigen(quat_idyntree).norm() <<  " ) " << std::endl;
        std::cerr << "Quaternion converted by KDL: " << quat_kdl.toString() << " ( norm " << toEigen(quat_kdl).norm() <<  " ) " << std::endl;
        std::cerr << "Quaternion converted by YARP: " << quat_yarp.toString() << " ( norm " << toEigen(quat_yarp).norm() <<  " ) " << std::endl;
        std::cerr << "Quaternion converted by Eigen: " << quat_eigen.toString() << " ( norm " << toEigen(quat_eigen).norm() <<  " ) " << std::endl;
    }

    // Only iDynTree and KDL share the same "positive real part" assumption for quaternions 
    ASSERT_EQUAL_VECTOR_TOL(quat_idyntree,quat_kdl,1e-7);
    /*
    ASSERT_EQUAL_VECTOR(quat_kdl,quat_yarp);
    ASSERT_EQUAL_VECTOR(quat_idyntree,quat_yarp);
    */

    if( verbose )
    {
        std::cerr << "Rotation converted back and forth from quaternion by iDynTree :\n" << rotBack_idyntree.toString() << std::endl;
        std::cerr << "Rotation converted back and forth from quaternion by KDL :\n" << rotBack_kdl.toString() << std::endl;
        std::cerr << "Rotation converted back and forth from quaternion by YARP :\n" << rotBack_yarp.toString() << std::endl;
        std::cerr << "Rotation converted back and forth from quaternion by Eigen:\n" << rotBack_eigen.toString() << std::endl;
    }

    ASSERT_EQUAL_MATRIX_TOL(randRot,rotBack_idyntree,1e-7);
    ASSERT_EQUAL_MATRIX_TOL(randRot,rotBack_kdl,1e-7);
    // Commented out until we sort out the problem in YARP matrix
    //ASSERT_EQUAL_MATRIX_TOL(randRot,rotBack_yarp,1e-7);
    ASSERT_EQUAL_MATRIX_TOL(randRot,rotBack_eigen,1e-7);


    // Check range
    helper.idyntree.update(quat_idyntree);
    helper.kdl.update(quat_kdl);
    helper.yarp.update(quat_yarp);
    helper.eigen.update(quat_eigen);

}


int main()
{
    std::cerr << "Quaternion Consistency check " << std::endl;

    // Check ranges of the output quaternions
    QuaternionMaxMinHelper rangeHelper;

    // First we check an easy one for debugging
    checkRotationToQuaternion(iDynTree::Rotation::RPY(M_PI,0.0,0.0),rangeHelper,false);

    // Then we check two rotation, to see if any implementation suffer of the
    // discontinuities reported in http://it.mathworks.com/matlabcentral/answers/164746-bug-in-dcm2quat-function
    iDynTree::Rotation T_positiveTrace = iDynTree::Rotation(0,-1,0,
                                                            1,0,0,
                                                            0,0,1);

    iDynTree::Rotation T_negativeTrace = iDynTree::Rotation(-1,0,0,
                                                             0,0,1,
                                                             0,1,0);

    checkRotationToQuaternion(T_positiveTrace,rangeHelper,true);
    checkRotationToQuaternion(T_negativeTrace,rangeHelper,true);

    // Then some random ones
    int nrOfTrials = 100000;


    for(int i=0; i < nrOfTrials;i++)
    {
        // Create random rotation
        iDynTree::Rotation randRot = iDynTree::getRandomRotation();

        checkRotationToQuaternion(randRot,rangeHelper,false);
    }

    std::cerr << "iDynTree quaternion ranges: " << std::endl;
    std::cerr << rangeHelper.idyntree.toString();

    std::cerr << "KDL quaternion ranges: " << std::endl;
    std::cerr << rangeHelper.kdl.toString();

    std::cerr << "YARP quaternion ranges: " << std::endl;
    std::cerr << rangeHelper.yarp.toString();

    std::cerr << "Eigen quaternion ranges: " << std::endl;
    std::cerr << rangeHelper.eigen.toString();


    return EXIT_SUCCESS;
}


