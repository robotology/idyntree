/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/EigenHelpers.h>
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

class RPYMaxMin
{
public:
    MaxMin help[3];

    void update(const iDynTree::Vector3 & rpy)
    {
        help[0].update(rpy(0));
        help[1].update(rpy(1));
        help[2].update(rpy(2));
    }

    std::string toString()
    {
        std::stringstream ss;
        ss << "Roll is in (" << iDynTree::rad2deg(help[0].min) << " , " << iDynTree::rad2deg(help[0].max) << ")" << std::endl;
        ss << "Pitch is in (" << iDynTree::rad2deg(help[1].min) << " , " << iDynTree::rad2deg(help[1].max) << ")" << std::endl;
        ss << "Yaw is in (" << iDynTree::rad2deg(help[2].min) << " , " << iDynTree::rad2deg(help[2].max) << ")" << std::endl;

        return ss.str();
    }
};

void getRPY(iDynTree::Rotation & rot,
            iDynTree::Vector3 & rpy_idyntree,
            iDynTree::Vector3 & rpy_kdl,
            iDynTree::Vector3 & rpy_yarp)
{
    rot.getRPY(rpy_idyntree(0),rpy_idyntree(1),rpy_idyntree(2));

    iDynTree::ToKDL(rot).GetRPY(rpy_kdl(0),rpy_kdl(1),rpy_kdl(2));

    yarp::sig::Matrix rotYarp(3,3);
    iDynTree::toYarp(rot,rotYarp);

    yarp::sig::Vector rpy_yarp_yarp(3);

    rpy_yarp_yarp = yarp::math::dcm2rpy(rotYarp);

    iDynTree::toiDynTree(rpy_yarp_yarp,rpy_yarp);
}

void RPYConsistencyCheck()
{
    std::cerr << "RPY Consistency check " << std::endl;

    RPYMaxMin kdl, idyntree, yarp;

    int nrOfTrials = 1000;

    for(int i=0; i < nrOfTrials;i++)
    {
        // Create random rotation
        iDynTree::Rotation randRot = iDynTree::getRandomRotation();

        iDynTree::Vector3 rpy_idyntree, rpy_kdl, rpy_yarp;
        getRPY(randRot,rpy_idyntree,rpy_kdl,rpy_yarp);

        idyntree.update(rpy_idyntree);
        kdl.update(rpy_kdl);
        yarp.update(rpy_yarp);

        ASSERT_EQUAL_VECTOR(rpy_kdl,rpy_kdl);


        iDynTree::Rotation rotCheck_idyntree = iDynTree::Rotation::RPY(rpy_idyntree(0),rpy_idyntree(1),rpy_idyntree(2));
        iDynTree::Rotation rotCheck_kdl = iDynTree::Rotation::RPY(rpy_kdl(0),rpy_kdl(1),rpy_kdl(2));
        iDynTree::Rotation rotCheck_yarp = iDynTree::Rotation::RPY(rpy_yarp(0),rpy_yarp(1),rpy_yarp(2));

        ASSERT_EQUAL_MATRIX(rotCheck_idyntree,rotCheck_kdl);
        ASSERT_EQUAL_MATRIX(rotCheck_idyntree,rotCheck_yarp);

    }

    // this should be r \in [-180,180], p \in [-90,90], y \in [-180,180]
    // this parametrization is consistent with YARP, KDL and XSens sensors
    std::cerr << "Check the range of RPY in iDynTree, YARP and KDL: " << std::endl;
    std::cerr << "iDynTree: " << std::endl;
    std::cerr << idyntree.toString();
    std::cerr << "KDL: " << std::endl;
    std::cerr << kdl.toString();
    std::cerr << "YARP: " << std::endl;
    std::cerr << yarp.toString();
}


// Extracted from https://github.com/robotology/yarp/blob/master/src/libYARP_dev/src/modules/msgs/ros/include/yarpRosHelper.h
// as this function is not exposed in public headers
inline bool convertEulerAngleYXZrads_to_quaternion(double *eulerXYZ, double *quaternion)
{
    bool ret = true;

    quaternion[0] = -sin(eulerXYZ[0]/2) * sin(eulerXYZ[1]/2) * sin(eulerXYZ[2]/2) + cos(eulerXYZ[0]/2) * cos(eulerXYZ[1]/2) * cos(eulerXYZ[2]/2);
    quaternion[1] =  sin(eulerXYZ[0]/2) * cos(eulerXYZ[1]/2) * cos(eulerXYZ[2]/2) + cos(eulerXYZ[0]/2) * sin(eulerXYZ[1]/2) * sin(eulerXYZ[2]/2);
    quaternion[2] = -sin(eulerXYZ[0]/2) * cos(eulerXYZ[1]/2) * sin(eulerXYZ[2]/2) + cos(eulerXYZ[0]/2) * sin(eulerXYZ[1]/2) * cos(eulerXYZ[2]/2);
    quaternion[3] =  sin(eulerXYZ[0]/2) * sin(eulerXYZ[1]/2) * cos(eulerXYZ[2]/2) + cos(eulerXYZ[0]/2) * cos(eulerXYZ[1]/2) * sin(eulerXYZ[2]/2);

    // Check that the norm is a unit vector
    double norm = 0;
    for(int i=0; i<4; i++)
    {
        norm += quaternion[i] * quaternion[i];
    }
    norm = sqrt(norm);

    if((norm -1)  >= 0.05)
    {
        std::cerr << "convertEulerAngleYXZrads_to_quaternion error: input quaternion is not a unit quaternion.\n";
        ret = false;
    }

    return ret;
}

void quatToRPY(iDynTree::Vector4 & quat,
               iDynTree::Vector3 & rpy_idyntree,
               iDynTree::Vector3 & rpy_kdl,
               iDynTree::Vector3 & rpy_yarp)
{
    iDynTree::Rotation rot = iDynTree::Rotation::RotationFromQuaternion(quat);

    rot.getRPY(rpy_idyntree(0),rpy_idyntree(1),rpy_idyntree(2));

    KDL::Rotation rotKDL = KDL::Rotation::Quaternion(quat(1),quat(2),quat(3),quat(0));

    rotKDL.GetRPY(rpy_kdl(0),rpy_kdl(1),rpy_kdl(2));

    yarp::sig::Vector quatYarp(4);
    iDynTree::toYarp(quat,quatYarp);

    yarp::sig::Vector rpy_yarp_yarp(3);

    rpy_yarp_yarp = yarp::math::dcm2rpy(yarp::math::SE3inv(yarp::math::quat2dcm(quatYarp)));

    iDynTree::toiDynTree(rpy_yarp_yarp,rpy_yarp);
}

void RPYToQuaternion(iDynTree::Vector3 & rpy,
                     iDynTree::Vector4 & quat_idyntree,
                     iDynTree::Vector4 & quat_kdl,
                     iDynTree::Vector4 & quat_yarp,
                     iDynTree::Vector4 & quat_yarp_dev)
{
    iDynTree::Rotation rot = iDynTree::Rotation::RPY(rpy(0),rpy(1),rpy(2));

    rot.getQuaternion(quat_idyntree);

    KDL::Rotation rotKDL = KDL::Rotation::RPY(rpy(0),rpy(1),rpy(2));

    rotKDL.GetQuaternion(quat_kdl(1),quat_kdl(2),quat_kdl(3),quat_kdl(0));

    yarp::sig::Vector rpyYarp(rpy.size());
    iDynTree::toYarp(rpy,rpyYarp);


    yarp::sig::Vector quatYarp = yarp::math::dcm2quat(yarp::math::SE3inv(yarp::math::rpy2dcm(rpyYarp)));

    iDynTree::toiDynTree(quatYarp,quat_yarp);

    yarp::sig::Vector quatYarpDev(4);

    convertEulerAngleYXZrads_to_quaternion(rpy.data(),quatYarpDev.data());

    // The yarp_dev uses the imaginary/real serializtion
    quat_yarp_dev(0) = quatYarpDev[3];
    quat_yarp_dev(1) = quatYarpDev[0];
    quat_yarp_dev(2) = quatYarpDev[1];
    quat_yarp_dev(3) = quatYarpDev[2];
}

/**
 * Function to check the quaternion ---> RPY
 * functions in iDynTree and YARP.
 * In particular YARP has known to have a problem in this, see :
 * https://github.com/robotology/yarp/issues/786
 */
void QuaternionToRPYConsistencyCheck()
{
    std::cerr << "QuaternionToRPY Consistency check " << std::endl;

    RPYMaxMin kdl, idyntree, yarp;

    int nrOfTrials = 1000;

    for(int i=0; i < nrOfTrials;i++)
    {
        // Create random quaternion
        iDynTree::Vector4 randQuat;
        for(int i=0; i < randQuat.size(); i++)
        {
            randQuat(i) = iDynTree::getRandomDouble(-1.0,1.0);
        }

        iDynTree::toEigen(randQuat).normalize();

        iDynTree::Vector3 rpy_idyntree, rpy_kdl, rpy_yarp;
        quatToRPY(randQuat,rpy_idyntree,rpy_kdl,rpy_yarp);

        idyntree.update(rpy_idyntree);
        kdl.update(rpy_kdl);
        yarp.update(rpy_yarp);

        ASSERT_EQUAL_VECTOR(rpy_kdl,rpy_kdl);


        iDynTree::Rotation rotCheck_idyntree = iDynTree::Rotation::RPY(rpy_idyntree(0),rpy_idyntree(1),rpy_idyntree(2));
        iDynTree::Rotation rotCheck_kdl = iDynTree::Rotation::RPY(rpy_kdl(0),rpy_kdl(1),rpy_kdl(2));
        iDynTree::Rotation rotCheck_yarp = iDynTree::Rotation::RPY(rpy_yarp(0),rpy_yarp(1),rpy_yarp(2));

        ASSERT_EQUAL_MATRIX(rotCheck_idyntree,rotCheck_kdl);
        ASSERT_EQUAL_MATRIX(rotCheck_idyntree,rotCheck_yarp);

    }

    // this should be r \in [-180,180], p \in [-90,90], y \in [-180,180]
    // this parametrization is consistent with YARP, KDL and XSens sensors
    std::cerr << "Check the range of RPY in iDynTree, YARP and KDL (when generated from quaternions): " << std::endl;
    std::cerr << "iDynTree: " << std::endl;
    std::cerr << idyntree.toString();
    std::cerr << "KDL: " << std::endl;
    std::cerr << kdl.toString();
    std::cerr << "YARP: " << std::endl;
    std::cerr << yarp.toString();
}


/**
 * Function to check the RPY --> Quaternion
 * functions in iDynTree and YARP.
 * In particular YARP has two version of this, one embedded in
 * YARP_dev that is not exposed and so we copied here.
 */
void RPYToQuaternionConsistencyCheck()
{
    std::cerr << "RPYToQuaternion Consistency check " << std::endl;

    int nrOfTrials = 1000;

    for(int i=0; i < nrOfTrials;i++)
    {
        // Create random rotation
        iDynTree::Vector3 randRPY;
        // The ranges for RPY (shared by iDynTree/KDL/YARP are -180,180, -90,90, -180,180
        randRPY(0) = iDynTree::getRandomDouble(-M_PI,M_PI);
        randRPY(1) = iDynTree::getRandomDouble(-M_PI/2,M_PI/2);
        randRPY(2) = iDynTree::getRandomDouble(-M_PI,M_PI);


        iDynTree::Vector4 quat_idyntree, quat_kdl, quat_yarp, quat_yarp_dev;
        RPYToQuaternion(randRPY,quat_idyntree,quat_kdl,quat_yarp,quat_yarp_dev);

        // Let's convert the quaternion back to rotation to compare it and avoid mismatch due to different choices of the quaternion
        iDynTree::Rotation rotCheck_idyntree = iDynTree::Rotation::RotationFromQuaternion(quat_idyntree);
        iDynTree::Rotation rotCheck_kdl = iDynTree::Rotation::RotationFromQuaternion(quat_kdl);
        iDynTree::Rotation rotCheck_yarp = iDynTree::Rotation::RotationFromQuaternion(quat_yarp);
        iDynTree::Rotation rotCheck_yarp_dev = iDynTree::Rotation::RotationFromQuaternion(quat_yarp_dev);

        std::cerr << "rotCheck_yarp :" << std::endl;
        std::cerr << rotCheck_yarp.toString() << std::endl;
        std::cerr << "rotCheck_yarp_dev :" << std::endl;
        std::cerr << rotCheck_yarp_dev.toString() << std::endl;

        ASSERT_EQUAL_MATRIX(rotCheck_idyntree,rotCheck_kdl);
        ASSERT_EQUAL_MATRIX(rotCheck_idyntree,rotCheck_yarp);
        // Commented out because https://github.com/robotology/yarp/issues/803
        // ASSERT_EQUAL_MATRIX(rotCheck_idyntree,rotCheck_yarp_dev);
    }

    std::cerr << "RPYToQuaternion Consistency check passed." << std::endl;

}

int main()
{
    RPYConsistencyCheck();
    QuaternionToRPYConsistencyCheck();
    RPYToQuaternionConsistencyCheck();

    return EXIT_SUCCESS;
}
