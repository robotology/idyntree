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
 
bool PredictSensorsMeasurements::makePrediction(const Model& model,const Traversal& traversal,const iDynTree::FreeFloatingPos& robotPos,const iDynTree::FreeFloatingVel& robotVel,iDynTree::FreeFloatingAcc& robotAcc,iDynTree::LinkPositions& linkPos,iDynTree::LinkVelArray& linkVel,iDynTree::LinkAccArray& linkAcc,const LinAcceleration& gravity,const iDynTree::SensorsList &sensorsList,iDynTree::SensorsMeasurements &predictedMeasurement)
{
    
   bool returnVal = true;

//     iDynTree::LinkPositions linkPos;linkPos.resize(model);
//     iDynTree::LinkVelArray linkVel;linkVel.resize(model);
//     iDynTree::LinkAccArray linkAcc;linkAcc.resize(model);
//     
//    std::cout<<"so far so good";
    iDynTree::AngAcceleration nullAngAccl;// = iDynTree::AngAcceleration::;
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
    int parentLinkId;
    iDynTree::LinAcceleration predictedAcc;
    iDynTree::AngVelocity predictedAngVel;
    
    //Iterate through each kind of accelrometer and find its parent. Compute local (classical accelration) 
    // It is automatically proper acceleration since gravity is incorporated into the base acceleration
    for(idx = 0; idx<numAccl; idx++)
    {
        accelerometer = (Accelerometer *)sensorsList.getSensor(iDynTree::ACCELEROMETER, idx);
        parentLinkId = accelerometer->getParentIndex();
        predictedAcc = accelerometer->predictMeasurement(linkAcc(parentLinkId),linkVel(parentLinkId));
//         predictedAcc = accelerometer->predictMeasurement(robotAcc.jointAcc(parentLinkId),robotVel.jointVel(parentLinkId));
        predictedMeasurement.setMeasurement(iDynTree::ACCELEROMETER,idx,predictedAcc);
    }
    for(idx = 0; idx<numGyro; idx++)
    {
        
        gyroscope = (Gyroscope*)sensorsList.getSensor(iDynTree::GYROSCOPE, idx);
        parentLinkId = accelerometer->getParentIndex();
        predictedAngVel = gyroscope->predictMeasurement(linkVel(parentLinkId));
//         predictedAngVel = gyroscope->predictMeasurement(robotVel.jointVel(parentLinkId));
        predictedMeasurement.setMeasurement(iDynTree::GYROSCOPE,idx,predictedAngVel);   
    }
    return(returnVal);
}

bool PredictSensorsMeasurements::makePrediction(const Model& model,const Traversal& traversal,const iDynTree::FreeFloatingPos& robotPos,const iDynTree::FreeFloatingVel& robotVel,iDynTree::FreeFloatingAcc& robotAcc,iDynTree::LinkPositions& linkPos,iDynTree::LinkVelArray& linkVel,iDynTree::LinkAccArray& linkAcc,const LinAcceleration& gravity,const iDynTree::SensorsList &sensorsList,iDynTree::VectorDynSize &predictedMeasurement)
{
    bool returnVal = true;
    // incorporating gravity into the base LinAcceleration
//     iDynTree::LinkPositions linkPos(model);
//     iDynTree::LinkVelArray linkVel(model);
//     iDynTree::LinkAccArray linkAcc(model);
    
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
    int parentLinkId;
    predictedMeasurement.resize(vecSize);
    LinAcceleration predictedAcc;
    AngVelocity predictedAngVel;
    for(int i =0; i<model.getNrOfLinks();i++)
    {
        // following the convention of BERDY, we iterate through the links from 
        // root to leaves and fill in the sensor info in each (accelero first and 
        // gyro second.
        
        if(acceleroMap[i] != NULL)
        {
        
            Accelerometer *accelerometer = (Accelerometer*)acceleroMap[i];
            parentLinkId = accelerometer->getParentIndex();
            predictedAcc = accelerometer->predictMeasurement(linkAcc(i),linkVel(i));


            parentLinkId = accelerometer->getParentIndex();
            predictedAcc = accelerometer->predictMeasurement(linkAcc(i),linkVel(i));
        
            predictedMeasurement.setVal(vecItr++,predictedAcc.getVal(0));
            predictedMeasurement.setVal(vecItr++,predictedAcc.getVal(1));
            predictedMeasurement.setVal(vecItr++,predictedAcc.getVal(2));
        }
        if(gyroMap[i] != NULL)
        {

            Gyroscope *gyroscope = (Gyroscope*)gyroMap[i];
            parentLinkId = gyroscope->getParentIndex();
            predictedAngVel = gyroscope->predictMeasurement(linkVel(i));
            predictedMeasurement.setVal(vecItr++,predictedAngVel.getVal(0));
            predictedMeasurement.setVal(vecItr++,predictedAngVel.getVal(1));
            predictedMeasurement.setVal(vecItr++,predictedAngVel.getVal(2));
        }
    }
    return(returnVal);

}

    
    

}