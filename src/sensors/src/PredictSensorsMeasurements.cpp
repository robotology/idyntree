/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Sensors/PredictSensorsMeasurements.h>
#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Sensors/AccelerometerSensor.h>
#include <iDynTree/Sensors/GyroscopeSensor.h>
#include <iDynTree/Sensors/ThreeAxisAngularAccelerometerSensor.h>
#include <iDynTree/Sensors/SixAxisForceTorqueSensor.h>
#include <iDynTree/Sensors/Sensors.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Dynamics.h>

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
                                const LinkNetExternalWrenches & externalWrenches,
                                      FreeFloatingAcc& buf_properRobotAcc,
                                      LinkPositions& buf_linkPos,
                                      LinkVelArray& buf_linkVel,
                                      LinkAccArray& buf_linkProperAcc,
                                      LinkInternalWrenches& buf_internalWrenches,
                                      FreeFloatingGeneralizedTorques& buf_outputTorques,
                                      SensorsMeasurements &predictedMeasurement)
{
    AngAcceleration nullAngAccl;
    nullAngAccl.zero();
    SpatialAcc gravityAccl(gravity,nullAngAccl);

    buf_properRobotAcc.baseAcc() = robotAcc.baseAcc() - gravityAccl;

    toEigen(buf_properRobotAcc.jointAcc()) = toEigen(robotAcc.jointAcc());

    // calling ForwardPosVelAccKinematics with arguments
    ForwardPosVelAccKinematics(model,traversal,robotPos,robotVel,
                               buf_properRobotAcc,buf_linkPos,buf_linkVel,buf_linkProperAcc);

    // Calling the backward pass of RNEA to simulat FT sensors
    RNEADynamicPhase(model,traversal,robotPos.jointPos(),buf_linkVel,buf_linkProperAcc,
                     externalWrenches,buf_internalWrenches,buf_outputTorques);

    return predictSensorsMeasurementsFromRawBuffers(model,sensorsList,traversal,
                                                    buf_linkVel,buf_linkProperAcc,
                                                    buf_internalWrenches,predictedMeasurement);
}

bool predictSensorsMeasurementsFromRawBuffers(const Model& /*model*/,
                                              const SensorsList& sensorsList,
                                              const Traversal& traversal,
                                              const LinkVelArray& buf_linkVel,
                                              const LinkAccArray& buf_linkProperAcc,
                                              const LinkInternalWrenches& buf_internalWrenches,
                                              SensorsMeasurements& predictedMeasurement)
{
    bool retVal = true;

    size_t numOfFTs = sensorsList.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);
    for(size_t idx = 0; idx<numOfFTs; idx++)
    {
        SixAxisForceTorqueSensor * ftSens = (SixAxisForceTorqueSensor*)sensorsList.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE, idx);

        Wrench predictedWrench = ftSens->predictMeasurement(traversal,buf_internalWrenches);
        retVal = retVal && predictedMeasurement.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,idx,predictedWrench);
    }


    //Iterate through each accelrometer and find its parent. Compute local (classical accelration)
    // It is automatically proper acceleration since gravity is incorporated into the base acceleration
    unsigned int numAccl = sensorsList.getNrOfSensors(iDynTree::ACCELEROMETER);
    for(size_t idx = 0; idx<numAccl; idx++)
    {
        AccelerometerSensor * accelerometer = (AccelerometerSensor *)sensorsList.getSensor(iDynTree::ACCELEROMETER, idx);
        LinkIndex parentLinkId = accelerometer->getParentLinkIndex();
        LinAcceleration predictedAcc = accelerometer->predictMeasurement(buf_linkProperAcc(parentLinkId),
                                                                         buf_linkVel(parentLinkId));
        retVal = retVal && predictedMeasurement.setMeasurement(iDynTree::ACCELEROMETER,idx,predictedAcc);
    }

    unsigned int numGyro = sensorsList.getNrOfSensors(iDynTree::GYROSCOPE);
    for(size_t idx = 0; idx<numGyro; idx++)
    {
        GyroscopeSensor * gyroscope = (GyroscopeSensor*)sensorsList.getSensor(iDynTree::GYROSCOPE, idx);
        LinkIndex parentLinkId = gyroscope->getParentLinkIndex();
        AngVelocity predictedAngVel = gyroscope->predictMeasurement(buf_linkVel(parentLinkId));
        retVal = retVal && predictedMeasurement.setMeasurement(iDynTree::GYROSCOPE,idx,predictedAngVel);
    }

    unsigned int numAngAccl = sensorsList.getNrOfSensors(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER);
    for(size_t idx = 0; idx<numAngAccl; idx++)
    {
        ThreeAxisAngularAccelerometerSensor * angAccelerometer =
        (ThreeAxisAngularAccelerometerSensor*)sensorsList.getSensor(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER, idx);
        LinkIndex parentLinkId = angAccelerometer->getParentLinkIndex();
        Vector3 predictedAngAcc = angAccelerometer->predictMeasurement(buf_linkProperAcc(parentLinkId));
        retVal = retVal && predictedMeasurement.setMeasurement(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER,idx,predictedAngAcc);
    }

    return retVal;
}





}
