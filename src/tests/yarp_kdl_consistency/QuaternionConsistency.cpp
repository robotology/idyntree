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
};

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

void checkRotationToQuaternion(const iDynTree::Rotation randRot, QuaternionMaxMinHelper & helper, bool verbose)
{
    iDynTree::Vector4 quat_idyntree, quat_kdl, quat_yarp;
    getQuaternion(randRot,quat_idyntree,quat_kdl,quat_yarp);

    if( verbose )
    {
        std::cerr << "Quaternion converted by KDL: " << quat_kdl.toString() << " ( norm " << toEigen(quat_kdl).norm() <<  " ) " << std::endl;
        std::cerr << "Quaternion converted by YARP: " << quat_yarp.toString() << " ( norm " << toEigen(quat_yarp).norm() <<  " ) " << std::endl;
        std::cerr << "Quaternion converted by iDynTree: " << quat_idyntree.toString() << " ( norm " << toEigen(quat_idyntree).norm() <<  " ) " << std::endl;
    }

    // ASSERT_EQUAL_VECTOR(quat_kdl,quat_yarp);
    // ASSERT_EQUAL_VECTOR(quat_idyntree,quat_kdl);
    // ASSERT_EQUAL_VECTOR(quat_idyntree,quat_yarp);


    // Check going back to a rotation
    iDynTree::Rotation randRotCheck = iDynTree::Rotation::RotationFromQuaternion(quat_idyntree);

    if( verbose )
    {
        std::cerr << "Original rotation:\n" << randRot.toString() << std::endl;
        std::cerr << "Rotation converted back and forth from quaternion:\n" << randRotCheck.toString() << std::endl;
    }
    // ASSERT_EQUAL_MATRIX_TOL(randRot,randRotCheck);

    // Check range
    helper.idyntree.update(quat_idyntree);
    helper.kdl.update(quat_kdl);
    helper.yarp.update(quat_yarp);

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
    iDynTree::Rotation T_1059 = iDynTree::Rotation(0.4218,-0.3220,0.8475,
                                                   0.7299,-0.4339,-0.5282,
                                                   0.5378,0.8414,0.0520);

    iDynTree::Rotation T_1060 = iDynTree::Rotation(0.4312,-0.2603,0.8639,
                                                   0.7289,-0.4637,-0.5036,
                                                   0.5317,0.8469,-0.0102);
                                                   
   iDynTree::Rotation T_fritz_1 = iDynTree::Rotation(0,-1,0,
                                                  1,0,0,
                                                  0,0,1);

    checkRotationToQuaternion(T_1059,rangeHelper,true);
    checkRotationToQuaternion(T_1060,rangeHelper,true);
    checkRotationToQuaternion(T_fritz_1,rangeHelper,true);

    // Then some random ones
    int nrOfTrials = 100000;


    for(int i=0; i < nrOfTrials;i++)
    {
        // Create random rotation
        iDynTree::Rotation randRot = iDynTree::getRandomRotation();

        checkRotationToQuaternion(randRot,rangeHelper,false);
    }

    std::cerr << "KDL quaternion ranges: " << std::endl;
    std::cerr << rangeHelper.kdl.toString();

    std::cerr << "YARP quaternion ranges: " << std::endl;
    std::cerr << rangeHelper.yarp.toString();


    return EXIT_SUCCESS;
}
