/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "testModels.h"

// KDL related includes
#include <kdl_codyco/position_loops.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include <kdl_codyco/KDLConversions.h>

#include <iDynTree/ModelIO/impl/urdf_import.hpp>

// iDynTree includes
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <iDynTree/Core/TestUtils.h>

#include <iDynTree/Core/EigenHelpers.h>

using namespace iDynTree;

void checkGetSetRPYIsIdempotent(double r, double p, double y)
{
    iDynTree::Rotation rotIDT = iDynTree::Rotation::RPY(r,p,y);
    double roll, pitch, yaw;
    rotIDT.getRPY(roll, pitch, yaw);

    iDynTree::Rotation rotIDTchecl = iDynTree::Rotation::RPY(roll,pitch,yaw);

    ASSERT_EQUAL_MATRIX(rotIDT,rotIDTchecl);
}

void checkSetRPYIsConsistentWithKDL(double r, double p, double y)
{
    KDL::Rotation rotKDL = KDL::Rotation::RPY(r,p,y);
    iDynTree::Rotation rotIDT = iDynTree::Rotation::RPY(r,p,y);

    ASSERT_EQUAL_MATRIX(rotIDT,ToiDynTree(rotKDL));
}

// Copied from https://github.com/ros/urdfdom/pull/67/files
// (that @traversaro wrote, so it is ok for copyright)
void checkRPY()
{
    std::vector<double> testAngles;
    testAngles.push_back(0.0);
    testAngles.push_back(M_PI/4);
    testAngles.push_back(M_PI/3);
    testAngles.push_back(M_PI/2);
    testAngles.push_back(M_PI);
    testAngles.push_back(-M_PI/4);
    testAngles.push_back(-M_PI/3);
    testAngles.push_back(-M_PI/2);
    testAngles.push_back(-M_PI);
    testAngles.push_back(1.0);
    testAngles.push_back(1.5);
    testAngles.push_back(2.0);
    testAngles.push_back(-1.0);
    testAngles.push_back(-1.5);
    testAngles.push_back(-2.0);

    for(size_t rIdx = 0; rIdx < testAngles.size(); rIdx++ )
    {
        for(size_t pIdx = 0; pIdx < testAngles.size(); pIdx++ )
        {
            for(size_t yIdx = 0; yIdx < testAngles.size(); yIdx++ )
            {

                checkGetSetRPYIsIdempotent(testAngles[rIdx],
                                           testAngles[pIdx],
                                           testAngles[yIdx]);


                checkSetRPYIsConsistentWithKDL(testAngles[rIdx],
                                               testAngles[pIdx],
                                               testAngles[yIdx]);
            }
        }
    }
}


int main()
{
    checkRPY();
    return EXIT_SUCCESS;
}
