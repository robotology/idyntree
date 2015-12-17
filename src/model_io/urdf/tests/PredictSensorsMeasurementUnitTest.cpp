/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Author: Naveen Kuppuswamy
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iostream>

# include <iDynTree/ModelIO/URDFGenericSensorsImport.h>
# include <iDynTree/Sensors/Sensors.h>
#include "testModels.h"
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/URDFModelImport.h>
#include <iDynTree/ModelIO/URDFDofsImport.h>
#include <iDynTree/Sensors/PredictSensorsMeasurements.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <iDynTree/Core/TestUtils.h>
const double acclTestVal = 1.5;
const double gyroTestVal = 1.5;
#include <cassert>
#include <cstdio>
#include <cstdlib>
using namespace iDynTree;

bool init(std::string fileName, Model &model, Traversal &traversal, SensorsList &sensorsList, SensorsMeasurements &predictedMeasurement)
{
    // load URDF model
    bool ok = modelFromURDF(fileName,model);
    ASSERT_EQUAL_DOUBLE(ok,true);
    std::cout<<"Model "<<fileName.c_str()<<" created with :"<<model.getNrOfDOFs()<<" DoFs"<<std::endl;

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(),2);
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(),1);
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(),1);
    ASSERT_EQUAL_DOUBLE(model.getNrOfFrames(),3);
    ASSERT_EQUAL_STRING(model.getLinkName(model.getDefaultBaseLink()),"link1");
   
    ok = model.computeFullTreeTraversal(traversal);

    ASSERT_EQUAL_DOUBLE(ok,true);
    
    // Load sensorList
    genericSensorsListFromURDF(fileName,sensorsList);

    ASSERT_EQUAL_DOUBLE(sensorsList.getNrOfSensors(ACCELEROMETER),2);
    ASSERT_EQUAL_DOUBLE(sensorsList.getNrOfSensors(GYROSCOPE),1);
    
    predictedMeasurement.setNrOfSensors(ACCELEROMETER,(sensorsList.getNrOfSensors(ACCELEROMETER)));
    predictedMeasurement.setNrOfSensors(GYROSCOPE,(sensorsList.getNrOfSensors(GYROSCOPE)));
    
}

bool runTest(const int& expID,const Model& model,const Traversal& traversal, const SensorsList& sensorsList, SensorsMeasurements& predictedMeasurement)
{     
    // quantities to be set according to experiment
    FreeFloatingPos robotPos(model);
    FreeFloatingVel robotVel(model);
    FreeFloatingAcc robotAcc(model);
    LinAcceleration gravity;
    
    iDynTree::LinkPositions linkPos(model);
    iDynTree::LinkVelArray linkVel(model);
    iDynTree::LinkAccArray linkAcc(model);
    
    PredictSensorsMeasurements predictedSensors;
    LinAcceleration accl1,accl2;
    AngVelocity gyro1;
    
    
    //experiments 1-accelerometer gravity test, 2-angularVelocity test, 3-angularAccelerationTest 
    
    std::cout<<"------------------------\n";
    std::cout<<"Experiment "<<expID<<"\n";
    switch(expID)
    {
        case 1 :gravity= LinearMotionVector3(0,0,9.8);
                robotAcc.baseAcc() = SpatialAcc::Zero();
                break;
            
        case 2 :gravity= LinearMotionVector3(0,0,0);
                robotAcc.baseAcc() = SpatialAcc::Zero();
                robotVel.jointVel()(0) = gyroTestVal;
                break;
        case 3 :gravity= LinearMotionVector3(0,0,0);
                robotAcc.baseAcc() = SpatialAcc::Zero();
                robotVel.jointVel()(0) = 0;          
                robotAcc.jointAcc()(0) = acclTestVal;
                break;
    }
    
    predictedSensors.makePrediction(model,traversal,robotPos,robotVel,robotAcc,linkPos,linkVel,linkAcc,gravity,sensorsList,predictedMeasurement);
    
    predictedMeasurement.getMeasurement(ACCELEROMETER,0,accl1);
    predictedMeasurement.getMeasurement(ACCELEROMETER,1,accl2);
    predictedMeasurement.getMeasurement(GYROSCOPE,0,gyro1);
    std::cout<<"Predicted Measurement (accl1): " <<accl1.toString()<<"\n";
    std::cout<<"Predicted Measurement (accl2): " <<accl2.toString()<<"\n";
    std::cout<<"Predicted Measurement (gyro1): " <<gyro1.toString()<<"\n";
    
    //checking obtained results
    switch(expID)
    {
        case 1 :ASSERT_EQUAL_DOUBLE(gyro1(0),0);
                ASSERT_EQUAL_DOUBLE(gyro1(1),0);
                ASSERT_EQUAL_DOUBLE(gyro1(2),0);
                ASSERT_EQUAL_DOUBLE(accl2(0),0);
                ASSERT_EQUAL_DOUBLE(accl2(1),0);
                ASSERT_EQUAL_DOUBLE(accl2(2),-9.8);
                break;
            
        case 2 :ASSERT_EQUAL_DOUBLE(gyro1(0),0);
                ASSERT_EQUAL_DOUBLE(gyro1(1),0);
                ASSERT_EQUAL_DOUBLE(gyro1(2),gyroTestVal);
                break;
        case 3 :ASSERT_EQUAL_DOUBLE(gyro1(0),0);
                ASSERT_EQUAL_DOUBLE(gyro1(1),0);
                ASSERT_EQUAL_DOUBLE(gyro1(2),0);
                ASSERT_EQUAL_DOUBLE(accl2(0),0);
                ASSERT_EQUAL_DOUBLE(accl2(1),-0.1*acclTestVal);
                ASSERT_EQUAL_DOUBLE(accl2(2),0);
                break;
    }
    Rotation::RPY(1,0,0);
  
}
int main()
{
    std::string fileName = getAbsModelPath("twoLinks.urdf");
    Model model;
    Traversal traversal;

    SensorsList sensorsList;
    SensorsMeasurements predictedMeasurement;
    init(fileName, model,traversal,sensorsList,predictedMeasurement);
    //experiments 1-accelerometer gravity test, 2-angularVelocity test, 3-angularAccelerationTest 
    
    for(int expID=1;expID<4;expID++)
    {
        runTest(expID,model,traversal,sensorsList,predictedMeasurement);
    }

    
    std::cout<<"Finished all three experiments\n";
    
    return 0;
}