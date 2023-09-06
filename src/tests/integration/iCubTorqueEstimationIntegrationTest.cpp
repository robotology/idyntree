// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "testModels.h"

#include <cmath>
#include <ctime>

#include <iDynTree/TestUtils.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/EigenSparseHelpers.h>
#include <iDynTree/ModelTestUtils.h>

#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>

#include <iDynTree/AccelerometerSensor.h>
#include <iDynTree/ThreeAxisAngularAccelerometerSensor.h>

#include <iDynTree/BerdyHelper.h>
#include <iDynTree/BerdySparseMAPSolver.h>
#include <iDynTree/ExtWrenchesAndJointTorquesEstimator.h>

using namespace iDynTree;

void extractJointTorquesAndContactForces(const iDynTree::BerdyHelper& berdyHelper,
                                         const iDynTree::VectorDynSize& jointPos,
                                         const iDynTree::VectorDynSize& estimatedDynamicVariables,
                                         const iDynTree::LinkUnknownWrenchContacts& unknownWrenches,
                                               iDynTree::VectorDynSize& estimatedJointTorques,
                                               iDynTree::LinkContactWrenches& contactWrenches)
{
    berdyHelper.extractJointTorquesFromDynamicVariables(estimatedDynamicVariables, jointPos, estimatedJointTorques);

    LinkNetExternalWrenches netExtWrenches(berdyHelper.model());

    berdyHelper.extractLinkNetExternalWrenchesFromDynamicVariables(estimatedDynamicVariables, netExtWrenches);

    estimateLinkContactWrenchesFromLinkNetExternalWrenches(berdyHelper.model(), unknownWrenches, netExtWrenches, contactWrenches);

    return;
}

struct CompareEstimatorsOptions
{
    bool removeAllFTSensors;
};

void setDiagonalMatrix(SparseMatrix<ColumnMajor>& mat, const IndexRange range, double val)
{
    for (int i=0; i < range.size; i++)
    {
        mat(range.offset+i, range.offset+i) = val;
    }
}

void compareEstimators(const std::string& urdfFileName,
                       const std::vector<std::string>& contactFrames,
                       const std::string& imuFrameName,
                       const CompareEstimatorsOptions& compareEstimatorsOptions)
{
    // Get the model we will use for testing
    std::string urdf_filename = getAbsModelPath(urdfFileName);

    // Let's use the full model
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(urdf_filename);

    ASSERT_IS_TRUE(ok);

    const Model& model = mdlLoader.model();

    iDynTree::FrameIndex  imuFrameIdx = model.getFrameIndex(imuFrameName);
    iDynTree::LinkIndex linkOfIMUIdx = model.getFrameLink(model.getFrameIndex(imuFrameName));
    std::string linkOfIMU = model.getLinkName(linkOfIMUIdx);
    iDynTree::Transform link_H_sensor = model.getFrameTransform(model.getFrameIndex(imuFrameName));

    // The sensors read by the model just contain the list of six axis F/T sensors
    // To have a fully equivalent situation, we need to add a three axis linear accelerometer
    // and a three axis angular accelerometer located in the imu_frame
    iDynTree::SensorsList usedSensors = mdlLoader.sensors();

    // First remove existing accelerometers
    usedSensors.removeAllSensorsOfType(ACCELEROMETER);
    usedSensors.removeAllSensorsOfType(THREE_AXIS_ANGULAR_ACCELEROMETER);
    usedSensors.removeAllSensorsOfType(GYROSCOPE);
    if (compareEstimatorsOptions.removeAllFTSensors)
    {
        usedSensors.removeAllSensorsOfType(SIX_AXIS_FORCE_TORQUE);
    }

    // Add accelerometers
    iDynTree::AccelerometerSensor linearAcc;
    linearAcc.setName("headLinearAccelerometer");
    linearAcc.setParentLink(linkOfIMU);
    linearAcc.setParentLinkIndex(linkOfIMUIdx);
    linearAcc.setLinkSensorTransform(link_H_sensor);
    int sensorIndex = usedSensors.addSensor(linearAcc);
    ASSERT_IS_TRUE(sensorIndex == 0);

    iDynTree::ThreeAxisAngularAccelerometerSensor angularAcc;
    angularAcc.setName("headAngularAccelerometer");
    angularAcc.setParentLink(linkOfIMU);
    angularAcc.setParentLinkIndex(linkOfIMUIdx);
    angularAcc.setLinkSensorTransform(link_H_sensor);
    sensorIndex = usedSensors.addSensor(angularAcc);
    ASSERT_IS_TRUE(sensorIndex == 0);

    // For now we assume that we have six f/t sensors and just the sensors that we added
    ASSERT_IS_TRUE(usedSensors.getNrOfSensors(iDynTree::ACCELEROMETER) == 1);
    ASSERT_IS_TRUE(usedSensors.getNrOfSensors(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER) == 1);

    // Build the two estimators
    iDynTree::ExtWrenchesAndJointTorquesEstimator determininsticEstimator;
    ok = determininsticEstimator.setModelAndSensors(mdlLoader.model(), usedSensors);
    ASSERT_IS_TRUE(ok);

    // Configure the BerdyHelper
    iDynTree::BerdyHelper berdyHelper;
    iDynTree::BerdyOptions berdyOptions;
    berdyOptions.berdyVariant = iDynTree::BERDY_FLOATING_BASE;

    // For now, we use all the external wrenches as unknown and we just add the fact that some are known to be zero
    // as sensors
    berdyOptions.includeAllNetExternalWrenchesAsSensors = true;
    berdyOptions.includeAllJointAccelerationsAsSensors  = true;
    ok = berdyHelper.init(model, usedSensors, berdyOptions);
    ASSERT_IS_TRUE(ok);

    iDynTree::BerdySparseMAPSolver probabilistEstimator(berdyHelper);
    probabilistEstimator.initialize();
    ASSERT_IS_TRUE(probabilistEstimator.isValid());

    // Check that the number of sensor is the expected one
    //
    int expectNumberOfMeasurements = 6*usedSensors.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) + // FT sensors
                                     3*usedSensors.getNrOfSensors(iDynTree::ACCELEROMETER) + // 1 accelerometer
                                     3*usedSensors.getNrOfSensors(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER) + // 1 angular accelrometer
                                     6*model.getNrOfLinks() + // external force/torques
                                     model.getNrOfDOFs(); // joint accelerations
    ASSERT_IS_TRUE(berdyHelper.getNrOfSensorsMeasurements() == expectNumberOfMeasurements);

    // We also make true we the number of unknows is equal to the number of ft sensors plus one
    ASSERT_EQUAL_DOUBLE(contactFrames.size(), usedSensors.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) + 1);

    // Create a simple list of unknown contacts
    iDynTree::LinkUnknownWrenchContacts unknownWrenchContacts(model);



    // Usual unknowns : one for each submodel
    for (const std::string& frame : contactFrames)
    {
        iDynTree::UnknownWrenchContact fullyUnknownContact;
        fullyUnknownContact.unknownType = FULL_WRENCH;
        fullyUnknownContact.contactPoint = iDynTree::Position::Zero();
        ASSERT_IS_TRUE(model.getFrameIndex(frame) != iDynTree::FRAME_INVALID_INDEX);
        ok = unknownWrenchContacts.addNewContactInFrame(model, model.getFrameIndex(frame), fullyUnknownContact);
        ASSERT_IS_TRUE(ok);
    }

    // Get random inputs
    iDynTree::JointPosDoubleArray  jointPos(model);
    iDynTree::JointDOFsDoubleArray jointVel(model);
    iDynTree::JointDOFsDoubleArray jointAcc(model);
    iDynTree::Vector3 imuAngularVel;
    iDynTree::SensorsMeasurements sensMeas(usedSensors);

    iDynTree::getRandomVector(jointPos);
    iDynTree::getRandomVector(jointVel);
    iDynTree::getRandomVector(jointAcc);

    iDynTree::getRandomVector(imuAngularVel);
    iDynTree::Vector3 imuLinAccReading;
    iDynTree::getRandomVector(imuLinAccReading);
    sensMeas.setMeasurement(iDynTree::ACCELEROMETER, 0, imuLinAccReading);
    iDynTree::Vector3 imuAngAccReading;
    iDynTree::getRandomVector(imuAngAccReading);
    sensMeas.setMeasurement(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER, 0, imuAngAccReading);
    for (int ftIdx = 0; ftIdx < usedSensors.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ftIdx++)
    {
        iDynTree::Vector3 buf3;
        iDynTree::Wrench ftMeas;
        iDynTree::getRandomVector(buf3);
        ftMeas.setLinearVec3(buf3);
        iDynTree::getRandomVector(buf3);
        ftMeas.setAngularVec3(buf3);
        sensMeas.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE, ftIdx, ftMeas);
    }

    // Compute estimation
    /// DETERMINISTIC ESTIMATOR
    LinkContactWrenches contactWrenchesEstimatedByDeterministic(model);
    JointDOFsDoubleArray jointTorquesEstimatedByDeterministic(model);

    std::clock_t tic = clock();
    ok = determininsticEstimator.updateKinematicsFromFloatingBase(jointPos, jointVel, jointAcc, imuFrameIdx,
                                                             imuLinAccReading, imuAngularVel, imuAngAccReading);
    ASSERT_IS_TRUE(ok);

    ok = determininsticEstimator.estimateExtWrenchesAndJointTorques(unknownWrenchContacts, sensMeas,
                                                                    contactWrenchesEstimatedByDeterministic, jointTorquesEstimatedByDeterministic);
    ASSERT_IS_TRUE(ok);
    std::clock_t toc = std::clock();
    double deterministicTime = (static_cast<double>(toc-tic))/CLOCKS_PER_SEC;

    /// PROBABILISTIC ESTIMATOR
    LinkContactWrenches contactWrenchesEstimatedByBerdy(model);
    JointDOFsDoubleArray jointTorquesEstimatedByBerdy(model);

    // Set the variances of the estimation problem

    // We should have enough measurements to solve the system, put a high covariance on the regulariation of the dynamics variable
    double highCovariance = 1e6;
    double lowCovariance  = 1e-6;

    // Dynamics regularization
    tic = clock();
    iDynTree::VectorDynSize dynamicsPriorMean(berdyHelper.getNrOfDynamicVariables());
    dynamicsPriorMean.zero();
    probabilistEstimator.setDynamicsRegularizationPriorExpectedValue(dynamicsPriorMean);
    iDynTree::Triplets dynamicsRegularizationCovarianceTriplets;
    dynamicsRegularizationCovarianceTriplets.setDiagonalMatrix(0, 0, highCovariance, berdyHelper.getNrOfDynamicVariables());
    iDynTree::SparseMatrix<iDynTree::ColumnMajor> dynamicsRegularizationCovariance(berdyHelper.getNrOfDynamicVariables(), berdyHelper.getNrOfDynamicVariables());
    dynamicsRegularizationCovariance.setFromTriplets(dynamicsRegularizationCovarianceTriplets);
    ASSERT_IS_TRUE(ok);
    probabilistEstimator.setDynamicsRegularizationPriorCovariance(dynamicsRegularizationCovariance);
    toc = clock();
    double timeToUpdateDynamicsRegularizationPrior = (static_cast<double>(toc-tic))/CLOCKS_PER_SEC;

    // std::cerr << "dynamicsRegularizationCovariance:\n" << toEigen(dynamicsRegularizationCovariance) << std::endl;


    // Dynamics contraint
    tic = std::clock();
    iDynTree::Triplets dynamicsConstraintCovarianceTriplets;
    dynamicsConstraintCovarianceTriplets.setDiagonalMatrix(0, 0, lowCovariance, berdyHelper.getNrOfDynamicEquations());
    iDynTree::SparseMatrix<iDynTree::ColumnMajor> dynamicsConstraintCovariance(berdyHelper.getNrOfDynamicEquations(), berdyHelper.getNrOfDynamicEquations());
    dynamicsConstraintCovariance.setFromTriplets(dynamicsConstraintCovarianceTriplets);
    probabilistEstimator.setDynamicsConstraintsPriorCovariance(dynamicsConstraintCovariance);
    toc = std::clock();
    double timeToUpdateDynamicsConstraintPrior = (static_cast<double>(toc-tic))/CLOCKS_PER_SEC;

    // std::cerr << "dynamicsConstraintCovariance:\n" << toEigen(dynamicsConstraintCovariance) << std::endl;


    // Measurements covariance and values
    tic = std::clock();
    iDynTree::VectorDynSize measures(berdyHelper.getNrOfSensorsMeasurements());
    tic = std::clock();

    // Known measurements all have low covariance
    iDynTree::IndexRange linAccRange = berdyHelper.getRangeSensorVariable(iDynTree::ACCELEROMETER, 0);
    iDynTree::setSubVector(measures, linAccRange, imuLinAccReading);
    setDiagonalMatrix(probabilistEstimator.measurementsPriorCovarianceInverse(), linAccRange, 1.0/lowCovariance);

    iDynTree::IndexRange angAccRange = berdyHelper.getRangeSensorVariable(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER, 0);
    iDynTree::setSubVector(measures, angAccRange, imuAngAccReading);
    setDiagonalMatrix(probabilistEstimator.measurementsPriorCovarianceInverse(), angAccRange, 1.0/lowCovariance);

    for (int ftIdx = 0; ftIdx < usedSensors.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ftIdx++)
    {
        iDynTree::IndexRange ftRange = berdyHelper.getRangeSensorVariable(iDynTree::SIX_AXIS_FORCE_TORQUE, ftIdx);
        iDynTree::Wrench measuredFt;
        sensMeas.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE, ftIdx, measuredFt);
        iDynTree::setSubVector(measures, ftRange, iDynTree::toEigen(measuredFt));
        setDiagonalMatrix(probabilistEstimator.measurementsPriorCovarianceInverse(), ftRange, 1.0/lowCovariance);
    }
    for (int dofIdx = 0; dofIdx < model.getNrOfDOFs(); dofIdx++)
    {
        iDynTree::IndexRange jointAccRange = berdyHelper.getRangeDOFSensorVariable(iDynTree::DOF_ACCELERATION_SENSOR, dofIdx);
        iDynTree::setSubVector(measures, jointAccRange, jointAcc(dofIdx));
        setDiagonalMatrix(probabilistEstimator.measurementsPriorCovarianceInverse(), jointAccRange, 1.0/lowCovariance);
    }

    // External wrench have low variance for all the wrenches that are not unknown, and high variance for the unknown one
    for (iDynTree::LinkIndex linkIdx = 0; linkIdx < model.getNrOfLinks(); linkIdx++)
    {
        // If there is an unknown on this link, estimate it
        bool linkUnknown = (unknownWrenchContacts.getNrOfContactsForLink(linkIdx) > 0);
        iDynTree::IndexRange extWrenchRange = berdyHelper.getRangeLinkSensorVariable(iDynTree::NET_EXT_WRENCH_SENSOR, linkIdx);
        if (linkUnknown)
        {
            setDiagonalMatrix(probabilistEstimator.measurementsPriorCovarianceInverse(), extWrenchRange, 1.0/highCovariance);

        }
        else
        {
            setDiagonalMatrix(probabilistEstimator.measurementsPriorCovarianceInverse(), extWrenchRange, 1.0/lowCovariance);
        }
        // Regardless of the covariance, the known value is 0
        iDynTree::Vector6 zeroVec;
        zeroVec.zero();
        iDynTree::setSubVector(measures, extWrenchRange, zeroVec);
    }

    toc = std::clock();
    double timeToUpdateMeasurementsPrior = (static_cast<double>(toc-tic))/CLOCKS_PER_SEC;


    // std::cerr << "measurementsCovariance:\n" << toEigen(measurementsCovariance) << std::endl;

    tic = clock();
    probabilistEstimator.updateEstimateInformationFloatingBase(jointPos, jointVel, imuFrameIdx, imuAngularVel, measures);
    ASSERT_IS_TRUE(ok);

    ok = probabilistEstimator.doEstimate();
    ASSERT_IS_TRUE(ok);
    toc = clock();
    double probabilisticTime = (static_cast<double>(toc-tic))/CLOCKS_PER_SEC;


    iDynTree::VectorDynSize estimatedDynamicVariables(berdyHelper.getNrOfDynamicVariables());
    probabilistEstimator.getLastEstimate(estimatedDynamicVariables);
    ASSERT_IS_TRUE(ok);

    // Convert the BerdyVector is something understandable
    extractJointTorquesAndContactForces(berdyHelper, jointPos, estimatedDynamicVariables, unknownWrenchContacts,
                                        jointTorquesEstimatedByBerdy, contactWrenchesEstimatedByBerdy);

    // Compare measurements
    ASSERT_EQUAL_VECTOR_REL_TOL(jointTorquesEstimatedByDeterministic, jointTorquesEstimatedByBerdy, 1e-4, 1e-6);

    std::cerr << "iCubTorqueEstimationIntegrationTest working fine for model " << urdfFileName << std::endl;
    std::cerr << "Number Of Dynamics Variables                 " << berdyHelper.getNrOfDynamicVariables() << std::endl;
    std::cerr << "Number Of Dynamics Equations                 " << berdyHelper.getNrOfDynamicEquations() << std::endl;
    std::cerr << "Number Of Measurements                       " << berdyHelper.getNrOfSensorsMeasurements() << std::endl;
    std::cerr << "Time to update dynamics variables prior      " << timeToUpdateDynamicsRegularizationPrior << std::endl;
    std::cerr << "Time to update dynamics equations prior      " << timeToUpdateDynamicsConstraintPrior << std::endl;
    std::cerr << "Time to update measurements prior            " << timeToUpdateMeasurementsPrior << std::endl;
    std::cerr << "Time for deterministic estimation            " << deterministicTime << std::endl;
    std::cerr << "Time for probabilistic estimation            " << probabilisticTime << std::endl;


    return;
}

void compareEstimators(const std::string& urdfFileName)
{
    // Let's use the full model
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(getAbsModelPath(urdfFileName));
    ASSERT_IS_TRUE(ok);

    // Get a random frame to use both as contact frame and imu frame
    const Model& model = mdlLoader.model();
    FrameIndex imuFrameIdx = model.getNrOfFrames()/2;
    ASSERT_IS_TRUE(imuFrameIdx != LINK_INVALID_INDEX);
    std::string imuFrameName = model.getFrameName(imuFrameIdx);

    std::vector<std::string> contactFrames = { imuFrameName };

    CompareEstimatorsOptions compareEstimatorsOptions;
    compareEstimatorsOptions.removeAllFTSensors = true;

    compareEstimators(urdfFileName, contactFrames, imuFrameName, compareEstimatorsOptions);
}

int main()
{
    std::string urdfFileName, imuFrameName;
    std::vector<std::string> contactFrameNames;

    // Test the case without FT sensors and just one unknown force
    CompareEstimatorsOptions compareEstimatorsOptions;
    compareEstimatorsOptions.removeAllFTSensors = true;

    // We can easily perform the test for all models in this easy case
    // However, this are commented out because in Debug mode under
    // valgrind this will take too much time
    /*
    std::cerr << "Testing BERDY without any internal FT sensor" << std::endl;
    for (unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        compareEstimators(std::string(IDYNTREE_TESTS_URDFS[mdl]));
    }*/

    urdfFileName = "twoLinks.urdf";
    contactFrameNames = {"link1"};
    imuFrameName = "link1";
    compareEstimators(urdfFileName, contactFrameNames, imuFrameName, compareEstimatorsOptions);

    urdfFileName = "twoLinks.urdf";
    contactFrameNames = {"link1"};
    imuFrameName = "link2";
    compareEstimators(urdfFileName, contactFrameNames, imuFrameName, compareEstimatorsOptions);

    urdfFileName = "twoLinks.urdf";
    contactFrameNames = {"link2"};
    imuFrameName = "link2";
    compareEstimators(urdfFileName, contactFrameNames, imuFrameName, compareEstimatorsOptions);

    urdfFileName = "icub_skin_frames.urdf";
    contactFrameNames = {"root_link"};
    imuFrameName = "imu_frame";
    compareEstimators(urdfFileName, contactFrameNames, imuFrameName, compareEstimatorsOptions);

    // Test the case with FT sensors
    std::cerr << "Testing BERDY with FT sensors" << std::endl;

    compareEstimatorsOptions.removeAllFTSensors = false;
    urdfFileName = "icub_skin_frames.urdf";
    contactFrameNames = {"root_link", "l_hand_dh_frame", "r_hand_dh_frame",
                         "l_lower_leg", "r_lower_leg", "r_foot_dh_frame", "l_foot_dh_frame"};
    imuFrameName = "imu_frame";
    compareEstimators(urdfFileName, contactFrameNames, imuFrameName, compareEstimatorsOptions);
}

