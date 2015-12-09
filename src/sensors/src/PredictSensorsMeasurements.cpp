/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Author: Naveen Kuppuswamy
 * email:  naveen.kuppuswamyt@iit.it
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
#include <map>
#include <iDynTree/Core/VectorDynSize.h>



namespace iDynTree {
PredictSensorsMeasurements::PredictSensorsMeasurements()
{
// All local variables, so nothing to do really.
}
PredictSensorsMeasurements::~PredictSensorsMeasurements()
{
// All local variables, so nothing to do really.
}
 
bool PredictSensorsMeasurements::operator()(const Model& model,const Traversal& traversal,const iDynTree::FreeFloatingPos& robotPos,const iDynTree::FreeFloatingVel& robotVel,iDynTree::FreeFloatingAcc& robotAcc,const LinAcceleration& gravity,const iDynTree::SensorsList &sensorsList,iDynTree::SensorsMeasurements &predictedMeasurement)
{
    
    bool returnVal = true;
    // incorporating gravity into the base LinAcceleration
    iDynTree::LinkPositions linkPos(model);
    iDynTree::LinkVelArray linkVel(model);
    iDynTree::LinkAccArray linkAcc(model);
    
    
    iDynTree::AngAcceleration nullAngAccl;
    iDynTree::SpatialAcc gravityAccl(gravity,nullAngAccl);
    
    robotAcc.baseAcc() = robotAcc.baseAcc() - gravityAccl;
    
    // calling ForwardPosVelAccKinematics with arguments
    ForwardPosVelAccKinematics(model,traversal,robotPos,robotVel,robotAcc,linkPos,linkVel,linkAcc);
    
    // looping though SensorList    
    unsigned int numAccl = sensorsList.getNrOfSensors(iDynTree::ACCELEROMETER);
    unsigned int numGyro = sensorsList.getNrOfSensors(iDynTree::GYROSCOPE);
    int idx;
    
   Accelerometer * accelerometer;
    Gyroscope * gyroscope;
    
    //Iterate through each kind of accelrometer and find its parent. Compute local (classical accelration) 
    // It is automatically proper acceleration since gravity is incorporated into the base acceleration
    for(idx = 0; idx<numAccl; idx++)
    {
        accelerometer = (Accelerometer *)sensorsList.getSensor(iDynTree::ACCELEROMETER, idx);
        int parentLinkId = accelerometer->getParentIndex();
        
        iDynTree::LinAcceleration measuredAcc, crossProdTerm;
        iDynTree::Transform link_T_sensor;
        iDynTree::SpatialAcc linkAccTerm;
        iDynTree::Twist localVelocity;
        returnVal = accelerometer->getLinkSensorTransform(link_T_sensor);
        localVelocity = link_T_sensor*linkVel(parentLinkId);
        linkAccTerm = link_T_sensor*(linkAcc(parentLinkId));
        crossProdTerm = (localVelocity.getAngularVec3()).cross(localVelocity.getLinearVec3());
        measuredAcc = (linkAccTerm).getLinearVec3() + crossProdTerm;       
        predictedMeasurement.setMeasurement(iDynTree::ACCELEROMETER,idx,measuredAcc);
        
    }
    for(idx = 0; idx<numGyro; idx++)
    {
        iDynTree::AngVelocity measuredAngVel;
        gyroscope = (Gyroscope*)sensorsList.getSensor(iDynTree::GYROSCOPE, idx);
        int parentLinkId = accelerometer->getParentIndex();
        iDynTree::Transform link_T_sensor;
        returnVal = gyroscope->getLinkSensorTransform(link_T_sensor);
        measuredAngVel = (link_T_sensor*linkVel(parentLinkId)).getAngularVec3();
        predictedMeasurement.setMeasurement(iDynTree::GYROSCOPE,idx,measuredAngVel);   
        
    }
    return(returnVal);
}

bool PredictSensorsMeasurements::operator()(const Model& model,const Traversal& traversal,const iDynTree::FreeFloatingPos& robotPos,const iDynTree::FreeFloatingVel& robotVel,iDynTree::FreeFloatingAcc& robotAcc,const LinAcceleration& gravity,const iDynTree::SensorsList &sensorsList,iDynTree::VectorDynSize &predictedMeasurement)
{
    bool returnVal = true;
    // incorporating gravity into the base LinAcceleration
    iDynTree::LinkPositions linkPos(model);
    iDynTree::LinkVelArray linkVel(model);
    iDynTree::LinkAccArray linkAcc(model);
    
    iDynTree::AngAcceleration nullAngAccl;
    iDynTree::SpatialAcc gravityAccl(gravity,nullAngAccl);
    
    robotAcc.baseAcc() = robotAcc.baseAcc() - gravityAccl;
    
    // calling ForwardPosVelAccKinematics with arguments
    ForwardPosVelAccKinematics(model,traversal,robotPos,robotVel,robotAcc,linkPos,linkVel,linkAcc);
    
    // looping though SensorList    
    unsigned int numAccl = sensorsList.getNrOfSensors(iDynTree::ACCELEROMETER);
    unsigned int numGyro = sensorsList.getNrOfSensors(iDynTree::GYROSCOPE);
    int idx;
    
    //Iterate through each kind of accelrometer and find its parent. Compute local (classical accelration) 
    // It is automatically proper acceleration since gravity is incorporated into the base acceleration

    
    std::map<unsigned int,Sensor*> acceleroMap;
    std::map<unsigned int,Sensor*> gyroMap;
    Sensor * sensor;
    for(idx = 0; idx<numAccl; idx++)
    {
        sensor = sensorsList.getSensor(iDynTree::ACCELEROMETER, idx);
        acceleroMap[sensor->getParentIndex()] = sensor;
    }
    for(idx = 0; idx<numGyro; idx++)
    {
        sensor = sensorsList.getSensor(iDynTree::GYROSCOPE, idx);
        gyroMap[sensor->getParentIndex()] = sensor;
    }
    
    unsigned int vecSize = (acceleroMap.size()+ gyroMap.size()) * 3;
    unsigned int vecItr = 0;
    predictedMeasurement.resize(vecSize);
    for(int i =0; i<model.getNrOfDOFs();i++)
    {
        // following the convention of BERDY, we iterate through the links from 
        // root to leaves and fill in the sensor info in each (accelero first and 
        // gyro second.
        
        if(acceleroMap[i] != NULL)
        {
            iDynTree::LinAcceleration measuredAcc, crossProdTerm;
            iDynTree::Transform link_T_sensor;
            iDynTree::SpatialAcc linkAccTerm;
            iDynTree::Twist localVelocity;
            Accelerometer *accelerometer = (Accelerometer*)acceleroMap[i];
            returnVal = accelerometer->getLinkSensorTransform(link_T_sensor);
            localVelocity = link_T_sensor*linkVel(i);
            linkAccTerm = link_T_sensor*(linkAcc(i));
            crossProdTerm = (localVelocity.getAngularVec3()).cross(localVelocity.getLinearVec3());
            measuredAcc = (linkAccTerm).getLinearVec3() + crossProdTerm;  
            predictedMeasurement.setVal(vecItr++,measuredAcc.getVal(0));
            predictedMeasurement.setVal(vecItr++,measuredAcc.getVal(1));
            predictedMeasurement.setVal(vecItr++,measuredAcc.getVal(2));
        }
        if(gyroMap[i] != NULL)
        {
            iDynTree::AngVelocity measuredAngVel;
            Gyroscope *gyroscope = (Gyroscope*)gyroMap[i];
            iDynTree::Transform link_T_sensor;
            returnVal = gyroscope->getLinkSensorTransform(link_T_sensor);
            measuredAngVel = (link_T_sensor*linkVel(i)).getAngularVec3();
            predictedMeasurement.setVal(vecItr++,measuredAngVel.getVal(0));
            predictedMeasurement.setVal(vecItr++,measuredAngVel.getVal(1));
            predictedMeasurement.setVal(vecItr++,measuredAngVel.getVal(2));
        }
    }
    return(returnVal);

}

    
    

}