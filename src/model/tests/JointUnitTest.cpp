// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Link.h>

#include <iDynTree/Position.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/Transform.h>
#include <iDynTree/TransformDerivative.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/SpatialMotionVector.h>
#include <iDynTree/MatrixFixSize.h>
#include <iDynTree/MatrixView.h>

#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/RevoluteSO2Joint.h>
#include <iDynTree/PrismaticJoint.h>
#include <iDynTree/SphericalJoint.h>

#include <iDynTree/EigenHelpers.h>

#include <cstdio>
#include <cstdlib>
#include <cmath>

using namespace iDynTree;

template<typename JointType>
bool getPerturbatedMatrices(const JointType & joint, const VectorDynSize& jointPos, const VectorDynSize& jointVel, const double dt,
                          const LinkIndex linkA, const LinkIndex linkB, Transform& perturbatedUpper, Transform& perturbatedLower)
{
    // We get the original value of the joint position and joint velocity
    Eigen::VectorXd theta = toEigen(jointPos).segment(joint.getPosCoordsOffset(), joint.getNrOfPosCoords());

    // And the original value of the joint velocity
    Eigen::VectorXd nuTheta = toEigen(jointVel).segment(joint.getDOFsOffset(), joint.getNrOfDOFs());

    VectorDynSize jointPosPerturbatedUpper = jointPos;
    VectorDynSize jointPosPerturbatedLower = jointPos;

    MatrixDynSize positionDerivative_J_velocity;
    positionDerivative_J_velocity.resize(joint.getNrOfPosCoords(), joint.getNrOfDOFs());
    MatrixView<double> jacobianView(positionDerivative_J_velocity);
    joint.getPositionDerivativeVelocityJacobian(jointPos, jacobianView);

    toEigen(jointPosPerturbatedUpper).segment(joint.getPosCoordsOffset(), joint.getNrOfPosCoords()) = theta+toEigen(positionDerivative_J_velocity)*nuTheta*dt/2;
    perturbatedUpper = joint.getTransform(jointPosPerturbatedUpper,linkA,linkB);

    toEigen(jointPosPerturbatedLower).segment(joint.getPosCoordsOffset(), joint.getNrOfPosCoords()) = theta-toEigen(positionDerivative_J_velocity)*nuTheta*dt/2;
    perturbatedLower = joint.getTransform(jointPosPerturbatedLower,linkA,linkB);

    return true;
}

template<typename JointType>
Matrix4x4 getHomTransformDerivative(const JointType & joint, const VectorDynSize& jointPos, const VectorDynSize& jointVel, const double dt,
                                    const LinkIndex linkA, const LinkIndex linkB)
{
    Matrix4x4 ret;

    iDynTree::Transform perturbatedUpperTrans, perturbatedLowerTrans;
    getPerturbatedMatrices(joint, jointPos, jointVel, dt, linkA, linkB, perturbatedUpperTrans, perturbatedLowerTrans);

    Matrix4x4 perturbatedLower = perturbatedLowerTrans.asHomogeneousTransform();
    Matrix4x4 perturbatedUpper = perturbatedUpperTrans.asHomogeneousTransform();

    toEigen(ret) = (toEigen(perturbatedUpper)-toEigen(perturbatedLower))/dt;

    return ret;
}

template<typename JointType>
Matrix6x6 getAdjTransformDerivative(const JointType & joint, const VectorDynSize& jointPos, const VectorDynSize& jointVel, const double dt,
                                    const LinkIndex linkA, const LinkIndex linkB)
{
    Matrix6x6 ret;

    iDynTree::Transform perturbatedUpperTrans, perturbatedLowerTrans;
    getPerturbatedMatrices(joint, jointPos, jointVel, dt, linkA, linkB, perturbatedUpperTrans, perturbatedLowerTrans);

    Matrix6x6 perturbatedLower = perturbatedLowerTrans.asAdjointTransform();
    Matrix6x6 perturbatedUpper = perturbatedUpperTrans.asAdjointTransform();

    toEigen(ret) = (toEigen(perturbatedUpper)-toEigen(perturbatedLower))/dt;

    return ret;
}

template<typename JointType>
Matrix6x6 getAdjTransformWrenchDerivative(const JointType & joint, const VectorDynSize& jointPos, const VectorDynSize& jointVel, const double dt,
                                          const LinkIndex linkA, const LinkIndex linkB)
{
    Matrix6x6 ret;

    iDynTree::Transform perturbatedUpperTrans, perturbatedLowerTrans;
    getPerturbatedMatrices(joint, jointPos, jointVel, dt, linkA, linkB, perturbatedUpperTrans, perturbatedLowerTrans);

    Matrix6x6 perturbatedLower = perturbatedLowerTrans.asAdjointTransformWrench();
    Matrix6x6 perturbatedUpper = perturbatedUpperTrans.asAdjointTransformWrench();

    toEigen(ret) = (toEigen(perturbatedUpper)-toEigen(perturbatedLower))/dt;

    return ret;
}

template<typename JointType>
void validateJointTransformDerivative(const JointType & joint, const VectorDynSize& jointPos, const VectorDynSize& jointVel,
                                      const LinkIndex linkA, const LinkIndex linkB)
{
    double numericalDerivStep = 1e-8;
    double tol = numericalDerivStep*1e2;

    Transform           trans    = joint.getTransform(jointPos,linkA,linkB);

    MatrixDynSize positionDerivative_J_velocity;
    positionDerivative_J_velocity.resize(joint.getNrOfPosCoords(), joint.getNrOfDOFs());
    MatrixView<double> jacobianView(positionDerivative_J_velocity);
    joint.getPositionDerivativeVelocityJacobian(jointPos, jacobianView);
    VectorDynSize jointPosDer(joint.getNrOfPosCoords());
    toEigen(jointPosDer) = toEigen(positionDerivative_J_velocity)*toEigen(jointVel);

    Matrix4x4 analyticMat;
    analyticMat.zero();
    for(size_t i = 0; i < joint.getNrOfPosCoords(); ++i)
    {
        toEigen(analyticMat) = toEigen(analyticMat) + toEigen(joint.getTransformDerivative(jointPos, linkA, linkB, i).asHomogeneousTransformDerivative())*jointPosDer(i);
    }

    TransformDerivative analytic;
    analytic.fromHomogeneousTransformDerivative(analyticMat);

    Matrix4x4 homTransformDerivAn = analytic.asHomogeneousTransformDerivative();
    Matrix4x4 homTransformDerivNum = getHomTransformDerivative(joint,jointPos,jointVel,numericalDerivStep,linkA,linkB);
    ASSERT_EQUAL_MATRIX_TOL(homTransformDerivAn,homTransformDerivNum,tol);

    Matrix6x6 adjTransformDerivAn = analytic.asAdjointTransformDerivative(trans);
    Matrix6x6 adjTransformDerivNum = getAdjTransformDerivative(joint,jointPos,jointVel,numericalDerivStep,linkA,linkB);
    ASSERT_EQUAL_MATRIX_TOL(adjTransformDerivAn,adjTransformDerivNum,tol);

    Matrix6x6 adjWrenchTransformDerivAn = analytic.asAdjointTransformWrenchDerivative(trans);
    Matrix6x6 adjWrenchTransformDerivNum = getAdjTransformWrenchDerivative(joint,jointPos,jointVel,numericalDerivStep,linkA,linkB);
    ASSERT_EQUAL_MATRIX_TOL(adjWrenchTransformDerivAn,adjWrenchTransformDerivNum,tol);
}

/**
 * This test validates the mathematical relationship between getTransform and
 * getMotionSubspaceVector methods as described in the theory.md document:
 *
 * {}^P H_C {}^C s_{P,C}(θ) ν_θ = {}^P \dot{H}_C
 *
 * where {}^P H_C is the transform from child to parent,
 * {}^C s_{P,C}(θ) is the motion subspace vector expressed in child frame,
 * and ν_θ is the joint velocity.
 */
template<typename JointType>
void validateJointMotionSubspaceMatrix(const JointType & joint, VectorDynSize& jointPos, VectorDynSize& jointVel,
                                      const LinkIndex parent, const LinkIndex child)
{
    // Small time step for numerical differentiation
    const double dt = 1e-6;
    const double tol = 1e-5;


    // Get current transform from child to parent at position theta
    Transform P_H_C = joint.getTransform(jointPos, parent, child);
    Matrix4x4 P_H_C_matrix = P_H_C.asHomogeneousTransform();

    // Get the motion subspace matrix

    // Calculate {}^P H_C * {}^C s * v (left side of the equation)
    // Convert the spatial motion vector to a 4x4 matrix using the wedge operator
    // Create a temporary Eigen vector to hold the spatial motion vector data
    Eigen::MatrixXd spatialMotionEigen;
    spatialMotionEigen.resize(6, joint.getNrOfDOFs());

    for(size_t i = 0; i < joint.getNrOfDOFs(); ++i)
    {
        // Get the motion subspace vector for each DOF
        spatialMotionEigen.col(i) = toEigen(joint.getMotionSubspaceVector(i, child, parent));
    }

    // Compute the spatial velocity by multiplying subspace with joint velocity
    Eigen::Matrix<double, 6, 1> C_s_vel = spatialMotionEigen * toEigen(jointVel).segment(joint.getDOFsOffset(), joint.getNrOfDOFs());


    // Apply the wedge operator using our helper function
    Matrix4x4 C_v_wedge;
    toEigen(C_v_wedge) = wedge6dTo4x4d(C_s_vel);

    // Calculate {}^P H_C * {}^C s * velocity
    Matrix4x4 left_side;
    toEigen(left_side) = toEigen(P_H_C_matrix) * toEigen(C_v_wedge);

    // Now calculate the right side: the numerical time derivative of the transform
    Matrix4x4 right_side = getHomTransformDerivative(joint, jointPos, jointVel, dt, parent, child);

    // Verify that {}^P H_C * {}^C s * jointVel = {}^P \dot{H}_C
    ASSERT_EQUAL_MATRIX_TOL(left_side, right_side, tol);
}

// Helper function to generate a random joint position independently of the joint type
template<typename JointType>
VectorDynSize setRandomJointPosition(const JointType& joint, VectorDynSize& jointPos)
{
    // Default implementation for generic joints, it is made generic by the normalization
    for (size_t i=0; i < joint.getNrOfPosCoords(); ++i)
    {
        // Generate a random value for each position coordinate
        double randomValue = getRandomDouble();
        jointPos(joint.getPosCoordsOffset() + i) = randomValue;
    }
    joint.normalizeJointPosCoords(jointPos);
    return jointPos;
}

// Template specialization for RevoluteSO2Joint
template<>
VectorDynSize setRandomJointPosition<RevoluteSO2Joint>(const RevoluteSO2Joint& joint, VectorDynSize& jointPos)
{
    // TODO: Fill in random position generation for RevoluteSO2Joint
    double randomAngle = getRandomDouble();
    jointPos(joint.getPosCoordsOffset()) = std::cos(randomAngle);
    jointPos(joint.getPosCoordsOffset() + 1) = std::sin(randomAngle);
    // Set the joint position from the angle
    joint.setJointPositionFromAngle(jointPos, randomAngle);
    return jointPos;
}

// Helper function to generate a random joint velocity independently of the joint type
template<typename JointType>
VectorDynSize setRandomJointVelocity(const JointType& joint, VectorDynSize& jointPos)
{
    // Default implementation for generic 1 DOF joints
    double randomValue = getRandomDouble();
    jointPos(joint.getDOFsOffset()) = randomValue;
    return jointPos;
}

template<typename JointType>
JointType getRandomJoint()
{
    // Create a random joint connecting links 0 and 1
    JointType joint(0, 1, getRandomTransform(), getRandomAxis());
    joint.setPosCoordsOffset(0);
    joint.setDOFsOffset(0);
    return joint;
}

// Specialization for SphericalJoint
template<>
SphericalJoint getRandomJoint<SphericalJoint>()
{
    SphericalJoint joint;
    joint.setAttachedLinks(0, 1);
    joint.setRestTransform(getRandomTransform());
    joint.setPosCoordsOffset(0);
    joint.setDOFsOffset(0);
    joint.setJointCenter(0, getRandomPosition());
    return joint;
}

/**
 * Test helper function that creates and tests a joint of a specific type
 * using both validation methods.
 */
template<typename JointType>
void testJoint(bool printProgress = false)
{
    for(unsigned int i=0; i < 10; i++)
    {
        // Create a random joint connecting links 0 and 1
        JointType joint = getRandomJoint<JointType>();

        VectorDynSize jointPos(joint.getNrOfPosCoords());
        setRandomJointPosition(joint, jointPos);

        VectorDynSize jointVel(joint.getNrOfDOFs());
        setRandomJointVelocity(joint, jointVel);

        // Test the joint transform derivatives in both directions
        validateJointTransformDerivative(joint, jointPos, jointVel,
                                      joint.getFirstAttachedLink(), joint.getSecondAttachedLink());
        validateJointTransformDerivative(joint, jointPos, jointVel,
                                      joint.getSecondAttachedLink(), joint.getFirstAttachedLink());

        // Test the relationship between transform and motion subspace in both directions
        validateJointMotionSubspaceMatrix(joint, jointPos, jointVel,
                                        joint.getFirstAttachedLink(), joint.getSecondAttachedLink());
        validateJointMotionSubspaceMatrix(joint, jointPos, jointVel,
                                        joint.getSecondAttachedLink(), joint.getFirstAttachedLink());

        if (printProgress) {
            std::cout << typeid(JointType).name() << " test " << i << " passed." << std::endl;
        }
    }
}


int main()
{
    // Test RevoluteJoint
    std::cout << "Testing RevoluteJoint..." << std::endl;
    testJoint<RevoluteJoint>();

    // Test PrismaticJoint
    std::cout << "Testing PrismaticJoint..." << std::endl;
    testJoint<PrismaticJoint>();

    // Test RevoluteSO2Joint
    std::cout << "Testing RevoluteSO2Joint..." << std::endl;
    testJoint<RevoluteSO2Joint>();

    // Test SphericalJoint
    std::cout << "Testing SphericalJoint..." << std::endl;
    testJoint<SphericalJoint>();

    std::cout << "All joint tests passed!" << std::endl;
    return EXIT_SUCCESS;
}
