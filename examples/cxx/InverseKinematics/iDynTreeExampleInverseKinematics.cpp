/**
* @ingroup idyntree_tutorials
*
* A tutorial on how to use the InverseKinematics class in iDynTree
*
* \author Silvio Traversaro
*
* CopyPolicy: Released under the terms of LGPL 2.0+ or later
*/

// C headers
#include <cmath>
#include <cstdlib>

// C++ headers
#include <string>
#include <vector>

// iDynTree headers
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/InverseKinematics.h>

int main(int argc, char *argv[])
{
    // Assume that the model is found in the current working directory
    // In production code, the model is tipical found using some strategy
    std::string modelFile = "kr16_2.urdf";

    // To make sure that the serialization of the joint is exactly the one that we need,
    // it is a good practice to explicitly specify the desired joint/degrees of freedom
    // serialization. Note that in production code this can be loaded from some parameter
    std::vector<std::string> consideredJoints
        = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"};


    // Helper class to load the model from an external format
    // Note: the loadReducedModelFromFile method permits to specify also a subset of the
    // joint of the robot, hence the name "reduced". As we are specifying all the joints
    // of the robot (we are just specifying them to make sure that the joint serialization
    // is the one that we desire) we are actually loading a "full" model
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadReducedModelFromFile(modelFile, consideredJoints);

    if (!ok) {
        std::cerr << "iDynTreeExampleInverseKinematics: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
    }

    // Create iDynTree::KinDynComputations, that can be used to compute the Forward Kinematics of robot
    // We compute the Forward kinematics position initially to get a reasonable starting point for a cartesian position,
    // and then to verify that the obtained Inverse Kinematics solution is actually closed to the desired one
    iDynTree::KinDynComputations kinDynMeas;
    ok = kinDynMeas.loadRobotModel(mdlLoader.model());

    if (!ok) {
        std::cerr << "iDynTreeExampleInverseKinematics: impossible to load model from " << modelFile << std::endl;
        return EXIT_FAILURE;
    }

    // As different frames are available on the robot, we need to explicitly specify the frames used as base
    // and as end-effector frames
    std::string baseFrame = "base_link";
    std::string endEffectorFrame = "tool0";

    // All units in iDynTree are the one of the International System of Units (SI)
    // Always pay attention to units, because other robotics software systems (such as YARP or KUKA)
    // actually use degrees for representing angles
    iDynTree::VectorDynSize jointPosInitialInRadians(mdlLoader.model().getNrOfPosCoords());
    // Let's start from the classical KUKA KR** home position
    double deg2rad = M_PI/180.0;
    jointPosInitialInRadians(0) =   0.0*deg2rad;
    jointPosInitialInRadians(1) = -90.0*deg2rad;
    jointPosInitialInRadians(2) =  90.0*deg2rad;
    jointPosInitialInRadians(3) =   0.0*deg2rad;
    jointPosInitialInRadians(4) =  90.0*deg2rad;
    jointPosInitialInRadians(5) =   0.0*deg2rad;

    // The iDynTree::KinDynComputations actually supports also setting the base position
    // and the base and joint velocity, but in this case we just use it for computing
    // the forward kinematics between two frames in the model, so we just need to
    kinDynMeas.setJointPos(jointPosInitialInRadians);

    // Let's read the initial cartesian position
    iDynTree::Transform base_H_ee_initial = kinDynMeas.getRelativeTransform(baseFrame, endEffectorFrame);

    // Print the initial joint and cartesian position
    std::cerr << "Initial cartesian linear position: " << std::endl << "\t" << base_H_ee_initial.getPosition().toString() << std::endl;
    std::cerr << "Initial joint position (radians): " << std::endl << "\t" << jointPosInitialInRadians.toString() << std::endl;

    // Let's define a new desired cartesian position, by decreasing the z-position of initial cartesian pose by 10 centimeters (0.1 meters):
    // We keep the same rotation
    iDynTree::Rotation initial_rot_pose = base_H_ee_initial.getRotation();
    // But we change the liner position
    iDynTree::Position initial_linear_pose = base_H_ee_initial.getPosition();
    initial_linear_pose(2) = initial_linear_pose(2) - 0.1;
    iDynTree::Transform base_H_ee_desired = iDynTree::Transform(initial_rot_pose, initial_linear_pose);


    // Let's print the desired position
    std::cerr << "Desired cartesian linear position: " << std::endl << "\t" << base_H_ee_desired.getPosition().toString() << std::endl;

    // Create a InverseKinematics class using the loaded model
    iDynTree::InverseKinematics ik;
    ok = ik.setModel(mdlLoader.model());

    if (!ok) {
        std::cerr << "iDynTreeExampleInverseKinematics: impossible to load the following model in a InverseKinematics class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return EXIT_FAILURE;
    }

    // Setup Inverse Kinematics problem

    // Set IK tolerances
    ik.setCostTolerance(0.0001);
    ik.setConstraintsTolerance(0.00001);

    // Always consider targets as costs, as opposed to a
    ik.setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintNone);

    // Use roll pitch yaw parametrization for the rotational part of the inverse kinematics
    ik.setRotationParametrization(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);

    // Note: the InverseKinematics class actually implements a floating base inverse kinematics,
    // meaning that both the joint position and the robot base are optimized to reach the desired cartesian position
    // As in this example we are considering instead the fixed-base case, we impose that the desired position of the base
    // is the identity
    iDynTree::Transform world_H_base = iDynTree::Transform::Identity();
    ok = ik.setFloatingBaseOnFrameNamed(baseFrame);
    if (!ok) {
        std::cerr << "iDynTreeExampleInverseKinematics: impossible to set floating base on " << baseFrame << std::endl;
        return EXIT_FAILURE;
    }
    ik.addFrameConstraint(baseFrame, world_H_base);

    // Add the desired task: the cartesian pose of the endEffectorFrame w.r.t to the baseFrame
    // Note that all the target are defined implicitly w.r.t. to the world frame of the ik, but
    // as we imposed that the transform between the baseFrame and the world is the identity,
    // asking for a desired world_H_ee pose is equivalent to asking for a desired base_H_ee pose
    ok = ik.addTarget(endEffectorFrame, base_H_ee_desired);
    if (!ok) {
        std::cerr << "iDynTreeExampleInverseKinematics: impossible to add target on  " << endEffectorFrame << std::endl;
        return EXIT_FAILURE;
    }

    // As the IK optimization problem is a non-linear iterative optimization, the optimization needs
    // an initial starting point, we use the initial position of the robot but note that this choice
    // can influence the actual convergence of the algorithm
    ik.setFullJointsInitialCondition(&world_H_base, &(jointPosInitialInRadians));

    // Actually run the inverse kinematics optimization problem
    ok = ik.solve();
    if (!ok) {
        std::cerr << "iDynTreeExampleInverseKinematics: impossible to solve inverse kinematics problem." << std::endl;
        return EXIT_FAILURE;
    }

    // Read the solution
    iDynTree::Transform basePoseOptimized;
    iDynTree::VectorDynSize jointPosOptimizedInRadians(mdlLoader.model().getNrOfPosCoords());

    ik.getFullJointsSolution(basePoseOptimized, jointPosOptimizedInRadians);

    // Compute the actual transform associated with the optimized values
    kinDynMeas.setJointPos(jointPosOptimizedInRadians);
    iDynTree::Transform base_H_ee_optimized = kinDynMeas.getRelativeTransform(baseFrame, endEffectorFrame);

    std::cerr << "Optimized cartesian position: " << std::endl << "\t" << base_H_ee_optimized.getPosition().toString() << std::endl;
    std::cerr << "Optimized joint position (radians): " << std::endl << "\t" << jointPosOptimizedInRadians.toString() << std::endl;

    return EXIT_SUCCESS;
}

