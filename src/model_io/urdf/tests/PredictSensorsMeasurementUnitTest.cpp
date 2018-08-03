/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iostream>

# include <iDynTree/Sensors/Sensors.h>
#include "testModels.h"
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
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

void init(std::string fileName, Model &model, Traversal &traversal,
          SensorsList &sensorsList, SensorsMeasurements &predictedMeasurement)
{
    // load URDF model
    ModelLoader loader;
    bool ok = loader.loadModelFromFile(fileName);
    model = loader.model();
    ASSERT_IS_TRUE(ok);
    std::cout<<"Model "<<fileName.c_str()<<" created with :"<<model.getNrOfDOFs()<<" DoFs"<<std::endl;

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(),2);
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(),1);
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(),1);
    ASSERT_EQUAL_DOUBLE(model.getNrOfFrames(),6);
    ASSERT_EQUAL_STRING(model.getLinkName(model.getDefaultBaseLink()),"link1");

    ok = model.computeFullTreeTraversal(traversal);

    ASSERT_EQUAL_DOUBLE(ok,true);

    // Load sensorList
    sensorsList = loader.sensors();

    ASSERT_EQUAL_DOUBLE(sensorsList.getNrOfSensors(ACCELEROMETER),2);
    ASSERT_EQUAL_DOUBLE(sensorsList.getNrOfSensors(GYROSCOPE),1);

    predictedMeasurement.resize(sensorsList);
}

void runTest(const int& expID,const Model& model,const Traversal& traversal,
             const SensorsList& sensorsList, SensorsMeasurements& predictedMeasurement)
{
    // quantities to be set according to experiment
    FreeFloatingPos robotPos(model);
    FreeFloatingVel robotVel(model);
    FreeFloatingAcc robotAcc(model);
    LinAcceleration gravity(0,0,0);
    LinkNetExternalWrenches externalWrenches(model);


    iDynTree::FreeFloatingAcc buf_properRobotAcc(model);
    iDynTree::LinkPositions buf_linkPos(model);
    iDynTree::LinkVelArray buf_linkVel(model);
    iDynTree::LinkAccArray buf_linkAcc(model);
    LinkInternalWrenches   buf_internalWrenches(model);
    FreeFloatingGeneralizedTorques buf_generalizedTorques(model);

    LinAcceleration accl1(0,0,0),accl2(0,0,0);
    AngVelocity gyro1(0,0,0);


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

    predictSensorsMeasurements(model,sensorsList,
                               traversal,robotPos,robotVel,robotAcc,gravity,externalWrenches,
                               buf_properRobotAcc,buf_linkPos,buf_linkVel,buf_linkAcc,buf_internalWrenches,
                               buf_generalizedTorques,predictedMeasurement);

    predictedMeasurement.getMeasurement(ACCELEROMETER,0,accl1);
    predictedMeasurement.getMeasurement(ACCELEROMETER,1,accl2);
    predictedMeasurement.getMeasurement(GYROSCOPE,0,gyro1);

    VectorDynSize measurementVect;
    bool ok = predictedMeasurement.toVector(measurementVect);
    std::cout<<"Predicted Measurement (accl1): " <<accl1.toString()<<"\n";
    std::cout<<"Predicted Measurement (accl2): " <<accl2.toString()<<"\n";
    std::cout<<"Predicted Measurement (gyro1): " <<gyro1.toString()<<"\n";
    std::cout<<"Predicted Measurement vectorised : "<<measurementVect.toString()<<"\n";
    ASSERT_IS_TRUE(ok);
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
                ASSERT_EQUAL_DOUBLE(accl2(1),0.1*acclTestVal);
                ASSERT_EQUAL_DOUBLE(accl2(2),0);
                break;
    }

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
