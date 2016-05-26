/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

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

int main()
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


    return EXIT_SUCCESS;
}
