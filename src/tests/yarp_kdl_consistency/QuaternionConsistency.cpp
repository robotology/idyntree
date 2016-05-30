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

#include <cstdlib>
#include <limits>
#include <iostream>
#include <sstream>

void getQuaternion(const iDynTree::Rotation & rot,
                   iDynTree::Vector4 & quat_idyntree,
                   iDynTree::Vector4 & quat_kdl,
                   iDynTree::Vector4 & quat_yarp)
{
    rot.getQuaternion(quat_idyntree);

    // Notice that KDL uses a imaginary/real serialization, while we use real/imaginary
    iDynTree::ToKDL(rot).GetQuaternion(quat_kdl(1),quat_kdl(2),quat_kdl(3),quat_kdl(0));

    yarp::sig::Matrix rotYarp(3,3);
    iDynTree::toYarp(rot,rotYarp);

    yarp::sig::Vector quat_yarp_yarp(4);

    quat_yarp_yarp = yarp::math::dcm2quat(rotYarp);

    iDynTree::toiDynTree(quat_yarp_yarp,quat_yarp);
}

void checkRotationToQuaternion(const iDynTree::Rotation randRot)
{
    iDynTree::Vector4 quat_idyntree, quat_kdl, quat_yarp;
    getQuaternion(randRot,quat_idyntree,quat_kdl,quat_yarp);

    std::cerr << "Quaternion converted by KDL: " << quat_kdl.toString() << " ( norm " << toEigen(quat_kdl).norm() <<  " ) " << std::endl;
    std::cerr << "Quaternion converted by YARP: " << quat_yarp.toString() << " ( norm " << toEigen(quat_yarp).norm() <<  " ) " << std::endl;
    std::cerr << "Quaternion converted by iDynTree: " << quat_idyntree.toString() << " ( norm " << toEigen(quat_idyntree).norm() <<  " ) " << std::endl;

    /*
    ASSERT_EQUAL_VECTOR(quat_kdl,quat_yarp);
    ASSERT_EQUAL_VECTOR(quat_idyntree,quat_kdl);
    ASSERT_EQUAL_VECTOR(quat_idyntree,quat_yarp);
    */

    // Check going back to a rotation
    iDynTree::Rotation randRotCheck = iDynTree::Rotation::RotationFromQuaternion(quat_idyntree);

    std::cerr << "Original rotation:\n" << randRot.toString() << std::endl;
    std::cerr << "Rotation converted back and forth from quaternion:\n" << randRotCheck.toString() << std::endl;


    //ASSERT_EQUAL_MATRIX(randRot,randRotCheck);
}

int main()
{
    std::cerr << "Quaternion Consistency check " << std::endl;


    // First we check an easy one for debugging
    checkRotationToQuaternion(iDynTree::Rotation::RPY(M_PI,0.0,0.0));

    // Then some random ones
    int nrOfTrials = 0;

    for(int i=0; i < nrOfTrials;i++)
    {
        // Create random rotation
        iDynTree::Rotation randRot = iDynTree::getRandomRotation();

        checkRotationToQuaternion(randRot);
    }

    return EXIT_SUCCESS;
}
