// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/PredictSensorsMeasurements.h>
#include <iDynTree/SpatialMotionVector.h>
#include <iDynTree/AccelerometerSensor.h>
#include <iDynTree/GyroscopeSensor.h>
#include <iDynTree/ThreeAxisAngularAccelerometerSensor.h>
#include <iDynTree/SixAxisForceTorqueSensor.h>
#include <iDynTree/Sensors.h>

#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/ForwardKinematics.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/Dynamics.h>

#include <iDynTree/SpatialAcc.h>
#include <iDynTree/EigenHelpers.h>

namespace iDynTree {

bool predictSensorsMeasurements(const Model & model,
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

    return predictSensorsMeasurementsFromRawBuffers(model,traversal,
                                                    buf_linkVel,buf_linkProperAcc,
                                                    buf_internalWrenches,predictedMeasurement);
}

bool predictSensorsMeasurementsFromRawBuffers(const Model& model,
                                              const Traversal& traversal,
                                              const LinkVelArray& buf_linkVel,
                                              const LinkAccArray& buf_linkProperAcc,
                                              const LinkInternalWrenches& buf_internalWrenches,
                                              SensorsMeasurements& predictedMeasurement)
{
    bool retVal = true;

    size_t numOfFTs = model.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);
    for(size_t idx = 0; idx<numOfFTs; idx++)
    {
        SixAxisForceTorqueSensor * ftSens = (SixAxisForceTorqueSensor*)model.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE, idx);

        Wrench predictedWrench = ftSens->predictMeasurement(traversal,buf_internalWrenches);
        retVal = retVal && predictedMeasurement.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,idx,predictedWrench);
    }


    //Iterate through each accelrometer and find its parent. Compute local (classical accelration)
    // It is automatically proper acceleration since gravity is incorporated into the base acceleration
    unsigned int numAccl = model.sensors().getNrOfSensors(iDynTree::ACCELEROMETER);
    for(size_t idx = 0; idx<numAccl; idx++)
    {
        AccelerometerSensor * accelerometer = (AccelerometerSensor *)model.sensors().getSensor(iDynTree::ACCELEROMETER, idx);
        LinkIndex parentLinkId = accelerometer->getParentLinkIndex();
        LinAcceleration predictedAcc = accelerometer->predictMeasurement(buf_linkProperAcc(parentLinkId),
                                                                         buf_linkVel(parentLinkId));
        retVal = retVal && predictedMeasurement.setMeasurement(iDynTree::ACCELEROMETER,idx,predictedAcc);
    }

    unsigned int numGyro = model.sensors().getNrOfSensors(iDynTree::GYROSCOPE);
    for(size_t idx = 0; idx<numGyro; idx++)
    {
        GyroscopeSensor * gyroscope = (GyroscopeSensor*)model.sensors().getSensor(iDynTree::GYROSCOPE, idx);
        LinkIndex parentLinkId = gyroscope->getParentLinkIndex();
        AngVelocity predictedAngVel = gyroscope->predictMeasurement(buf_linkVel(parentLinkId));
        retVal = retVal && predictedMeasurement.setMeasurement(iDynTree::GYROSCOPE,idx,predictedAngVel);
    }

    unsigned int numAngAccl = model.sensors().getNrOfSensors(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER);
    for(size_t idx = 0; idx<numAngAccl; idx++)
    {
        ThreeAxisAngularAccelerometerSensor * angAccelerometer =
        (ThreeAxisAngularAccelerometerSensor*)model.sensors().getSensor(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER, idx);
        LinkIndex parentLinkId = angAccelerometer->getParentLinkIndex();
        Vector3 predictedAngAcc = angAccelerometer->predictMeasurement(buf_linkProperAcc(parentLinkId));
        retVal = retVal && predictedMeasurement.setMeasurement(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER,idx,predictedAngAcc);
    }

    return retVal;
}

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
    Model modelCopy = model;
    modelCopy.sensors() = sensorsList;

    return predictSensorsMeasurements(modelCopy, traversal, robotPos, robotVel, robotAcc, gravity, 
                                      externalWrenches, buf_properRobotAcc, buf_linkPos, buf_linkVel, 
                                      buf_linkProperAcc, buf_internalWrenches, buf_outputTorques, predictedMeasurement);
}

bool predictSensorsMeasurementsFromRawBuffers(const Model& model,
                                              const SensorsList& sensorsList,
                                              const Traversal& traversal,
                                              const LinkVelArray& buf_linkVel,
                                              const LinkAccArray& buf_linkProperAcc,
                                              const LinkInternalWrenches& buf_internalWrenches,
                                              SensorsMeasurements& predictedMeasurement)
{
    Model modelCopy = model;
    modelCopy.sensors() = sensorsList;

    return predictSensorsMeasurementsFromRawBuffers(modelCopy, traversal, buf_linkVel, 
                                                    buf_linkProperAcc, buf_internalWrenches, predictedMeasurement);
}





}
