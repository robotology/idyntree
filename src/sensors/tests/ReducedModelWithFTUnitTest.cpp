#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/SixAxisForceTorqueSensor.h>
#include <iDynTree/Sensors/ModelSensorsTransformers.h>
#include <iDynTree/Model/ModelTransformers.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include "testModels.h"
#include <iostream>
#include <memory>

using namespace std;
using namespace iDynTree;

std::vector<std::string> get_iCubJointsSensorised()
{
    std::vector<std::string> consideredJoints;

    // Joints
    consideredJoints.push_back("torso_pitch");
    consideredJoints.push_back("torso_roll");
    consideredJoints.push_back("torso_yaw");
    consideredJoints.push_back("neck_pitch");
    consideredJoints.push_back("neck_roll");
    consideredJoints.push_back("neck_yaw");
    consideredJoints.push_back("l_shoulder_pitch");
    consideredJoints.push_back("l_shoulder_roll");
    consideredJoints.push_back("l_shoulder_yaw");
    consideredJoints.push_back("l_elbow");
    consideredJoints.push_back("r_shoulder_pitch");
    consideredJoints.push_back("r_shoulder_roll");
    consideredJoints.push_back("r_shoulder_yaw");
    consideredJoints.push_back("r_elbow");
    consideredJoints.push_back("l_hip_pitch");
    consideredJoints.push_back("l_hip_roll");
    consideredJoints.push_back("l_hip_yaw");
    consideredJoints.push_back("l_knee");
    consideredJoints.push_back("l_ankle_pitch");
    consideredJoints.push_back("l_ankle_roll");
    consideredJoints.push_back("r_hip_pitch");
    consideredJoints.push_back("r_hip_roll");
    consideredJoints.push_back("r_hip_yaw");
    consideredJoints.push_back("r_knee");
    consideredJoints.push_back("r_ankle_pitch");
    consideredJoints.push_back("r_ankle_roll");

    // FTs
    consideredJoints.push_back("l_leg_ft_sensor");
    consideredJoints.push_back("r_leg_ft_sensor");
    consideredJoints.push_back("l_foot_ft_sensor");
    consideredJoints.push_back("r_foot_ft_sensor");
    consideredJoints.push_back("l_arm_ft_sensor");
    consideredJoints.push_back("r_arm_ft_sensor");

    return consideredJoints;
}

Model getModel(const std::string& fileName)
{
    ModelLoader loader;
    bool ok = loader.loadModelFromFile(fileName);
    // Load model
    ASSERT_IS_TRUE(ok);
    Model model = loader.model();

    return model;
}

SensorsList getSensors(const std::string& fileName)
{
    ModelLoader loader;
    bool ok = loader.loadModelFromFile(fileName);
    // Load model
    ASSERT_IS_TRUE(ok);
    SensorsList sensorList = loader.sensors();

    return sensorList;
}

struct DataFT {
    string jointName;
    // Links
    string parent_linkName;
    string child_linkName;
    string grandparent_linkName;
    string grandchild_linkName;
    // Joints
    string grandparentToParent_jointName;
    string childToGranchild_jointName;
};

struct ExperimentFT {
    DataFT dataFT;
    vector<string> removedJoints;
    string expectedFirstLinkName;
    string expectedSecondLinkName;
};

vector<string> removeJoints(const vector<string>& originalVector,
                            const vector<string>& jointsToBeRemoved)
{
    vector<string> reducedJointList = originalVector;

    for (auto joint : jointsToBeRemoved) {
        auto it = std::find(reducedJointList.begin(), reducedJointList.end(), joint);
        if(it != reducedJointList.end()) {
            cout << "Removing: " << joint << endl;
            reducedJointList.erase(it);
        }
    }

    return reducedJointList;
}

void printFT(const SixAxisForceTorqueSensor& ft)
{
    cout << "Sensor Name:\t" << ft.getName() << endl;
    cout << "Is Valid: \t"   << ft.isValid() << endl;
    cout << "First Link:\t"  << ft.getFirstLinkName() << endl;
    cout << "Second Link:\t" << ft.getSecondLinkName() << endl;
}

int main()
{
    // Load data
    std::string fileName = getAbsModelPath("icub_sensorised.urdf");
    Model fullModel = getModel(fileName);
    SensorsList fullSensors = getSensors(fileName);

    // Store data for a given FT sensor
    DataFT dataFT_l_leg;
    dataFT_l_leg.jointName = "l_leg_ft_sensor";
    // Links
    dataFT_l_leg.parent_linkName = "l_hip_2";
    dataFT_l_leg.child_linkName = "l_hip_3";
    dataFT_l_leg.grandparent_linkName = "l_hip_1";
    dataFT_l_leg.grandchild_linkName = "l_upper_leg";
    // Joints
    dataFT_l_leg.grandparentToParent_jointName = "l_hip_roll";
    dataFT_l_leg.childToGranchild_jointName = "l_hip_yaw";

    // Get all the iCub joints
    vector<string> fullJoints = get_iCubJointsSensorised();

    // Keep only one FT
    vector<string> removedJoints;
    removedJoints.push_back("r_leg_ft_sensor");
    removedJoints.push_back("l_foot_ft_sensor");
    removedJoints.push_back("r_foot_ft_sensor");
    removedJoints.push_back("l_arm_ft_sensor");
    removedJoints.push_back("r_arm_ft_sensor");
    vector<string> fullJoints_1FT = removeJoints(fullJoints, removedJoints);

    // Setup the experiments
    // ---------------------

    // 1) The reduced model doesn't lump links which the FT is connected
    ExperimentFT test_defaultModel;
    test_defaultModel.dataFT = dataFT_l_leg;
    test_defaultModel.expectedFirstLinkName = dataFT_l_leg.parent_linkName;
    test_defaultModel.expectedSecondLinkName = dataFT_l_leg.child_linkName;

    // 2) The reduced model doesn't contain the firstLink
    ExperimentFT test_removeFirstLink;
    test_removeFirstLink.dataFT = dataFT_l_leg;
    test_removeFirstLink.removedJoints.push_back(test_removeFirstLink.dataFT.grandparentToParent_jointName);
    test_removeFirstLink.expectedFirstLinkName = dataFT_l_leg.grandparent_linkName;
    test_removeFirstLink.expectedSecondLinkName = dataFT_l_leg.child_linkName;

    // 3) The reduced model doesn't contain the secondLink
    ExperimentFT test_removeSecondLink;
    test_removeSecondLink.dataFT = dataFT_l_leg;
    test_removeSecondLink.removedJoints.push_back(test_removeSecondLink.dataFT.childToGranchild_jointName);
    test_removeSecondLink.expectedFirstLinkName = dataFT_l_leg.parent_linkName;
    test_removeSecondLink.expectedSecondLinkName = dataFT_l_leg.child_linkName;

    // 3) The reduced model doesn't contain both firstLink and secondLink
    ExperimentFT test_removeFirstAndSecondLink;
    test_removeFirstAndSecondLink.dataFT = dataFT_l_leg;
    test_removeFirstAndSecondLink.removedJoints.push_back(test_removeFirstAndSecondLink.dataFT.grandparentToParent_jointName);
    test_removeFirstAndSecondLink.removedJoints.push_back(test_removeFirstAndSecondLink.dataFT.childToGranchild_jointName);
    test_removeFirstAndSecondLink.expectedFirstLinkName = dataFT_l_leg.grandparent_linkName;
    test_removeFirstAndSecondLink.expectedSecondLinkName = dataFT_l_leg.child_linkName;

    // Group all the experiments together
    vector<ExperimentFT> experiments;
    experiments.push_back(test_defaultModel);
    experiments.push_back(test_removeFirstLink);
    experiments.push_back(test_removeSecondLink);
    experiments.push_back(test_removeFirstAndSecondLink);

    for (auto experiment : experiments)
    {
        bool ok;

        Model reducedModel;
        SensorsList reducedSensors;

        ok = createReducedModelAndSensors(fullModel,
                                          fullSensors,
                                          removeJoints(fullJoints_1FT, experiment.removedJoints),
                                          reducedModel,
                                          reducedSensors);
        ASSERT_IS_TRUE(ok);
        ASSERT_EQUAL_DOUBLE(reducedSensors.getNrOfSensors(SIX_AXIS_FORCE_TORQUE), 1);

        Sensor* s = reducedSensors.getSensor(SIX_AXIS_FORCE_TORQUE, 0);
        ASSERT_IS_TRUE(s != nullptr);
        ASSERT_IS_TRUE(s->isValid());

        JointSensor* jointSens = dynamic_cast<JointSensor*>(s);
        ASSERT_IS_TRUE(jointSens != nullptr);

        unique_ptr<SixAxisForceTorqueSensor> sensorCopy;
        sensorCopy.reset(static_cast<SixAxisForceTorqueSensor*>(jointSens->clone()));
        ASSERT_IS_TRUE(sensorCopy != nullptr);

        printFT(*sensorCopy);
        cout << endl;

        ASSERT_EQUAL_STRING(sensorCopy->getFirstLinkName(), experiment.expectedFirstLinkName);
        ASSERT_EQUAL_STRING(sensorCopy->getSecondLinkName(), experiment.expectedSecondLinkName);

        // Test transform
        // --------------

        // Get the transform from the fixed joint.
        // It uses the createReducedModel() logic. It is used as ground truth.
        Transform parent_H_child;
        auto jointFT = reducedModel.getJoint(reducedModel.getJointIndex(experiment.dataFT.jointName));
        parent_H_child = jointFT->getRestTransform(jointFT->getFirstAttachedLink(),
                                                   jointFT->getSecondAttachedLink());

        // Get the transform from the sensor.
        // This is calculated by createReducedModelAndSensors() and it is the method under test.
        LinkIndex firstLinkIndex = sensorCopy->getFirstLinkIndex();
        LinkIndex secondLinkIndex = sensorCopy->getSecondLinkIndex();
        Transform firstLink_H_sensorFrame;
        Transform secondLink_H_sensorFrame;
        ok = true;
        ok = ok && sensorCopy->getLinkSensorTransform(firstLinkIndex,  firstLink_H_sensorFrame);
        ok = ok && sensorCopy->getLinkSensorTransform(secondLinkIndex, secondLink_H_sensorFrame);
        ASSERT_IS_TRUE(ok);

        ASSERT_EQUAL_TRANSFORM(parent_H_child,
                               firstLink_H_sensorFrame*secondLink_H_sensorFrame.inverse());
    }

    return EXIT_SUCCESS;
}
