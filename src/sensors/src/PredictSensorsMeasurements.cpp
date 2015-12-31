/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Author: Naveen Kuppuswamy
 * email:  naveen.kuppuswamy@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <iDynTree/Sensors/PredictSensorsMeasurements.h>
#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Sensors/Accelerometer.h>
#include <iDynTree/Sensors/Gyroscope.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree {


bool predictSensorsMeasurements(const Model & model,
                                const SensorsList &sensorsList,
                                const Traversal & traversal,
                                const FreeFloatingPos& robotPos,
                                const FreeFloatingVel& robotVel,
                                const FreeFloatingAcc& robotAcc,
                                const LinAcceleration & gravity,
                                      FreeFloatingAcc& buf_properRobotAcc,
                                      LinkPositions& buf_linkPos,
                                      LinkVelArray& buf_linkVel,
                                      LinkAccArray& buf_linkProperAcc,
                                      SensorsMeasurements &predictedMeasurement)
{

    bool returnVal = true;

    AngAcceleration nullAngAccl;
    nullAngAccl.zero();
    SpatialAcc gravityAccl(gravity,nullAngAccl);

    buf_properRobotAcc.baseAcc() = robotAcc.baseAcc() - gravityAccl;

    toEigen(buf_properRobotAcc.jointAcc()) = toEigen(robotAcc.jointAcc());

    // calling ForwardPosVelAccKinematics with arguments
    ForwardPosVelAccKinematics(model,traversal,robotPos,robotVel,
                               buf_properRobotAcc,buf_linkPos,buf_linkVel,buf_linkProperAcc);

    // looping though SensorList

    //Iterate through each accelrometer and find its parent. Compute local (classical accelration)
    // It is automatically proper acceleration since gravity is incorporated into the base acceleration
    unsigned int numAccl = sensorsList.getNrOfSensors(iDynTree::ACCELEROMETER);
    for(size_t idx = 0; idx<numAccl; idx++)
    {
        Accelerometer * accelerometer = (Accelerometer *)sensorsList.getSensor(iDynTree::ACCELEROMETER, idx);
        LinkIndex parentLinkId = accelerometer->getParentIndex();
        LinAcceleration predictedAcc = accelerometer->predictMeasurement(buf_linkProperAcc(parentLinkId),
                                                                         buf_linkVel(parentLinkId));
        predictedMeasurement.setMeasurement(iDynTree::ACCELEROMETER,idx,predictedAcc);
    }

    unsigned int numGyro = sensorsList.getNrOfSensors(iDynTree::GYROSCOPE);
    for(size_t idx = 0; idx<numGyro; idx++)
    {
        Gyroscope * gyroscope = (Gyroscope*)sensorsList.getSensor(iDynTree::GYROSCOPE, idx);
        LinkIndex parentLinkId = gyroscope->getParentIndex();
        AngVelocity predictedAngVel = gyroscope->predictMeasurement(buf_linkVel(parentLinkId));
        predictedMeasurement.setMeasurement(iDynTree::GYROSCOPE,idx,predictedAngVel);
    }

    return returnVal;
}




}
