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

#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/PrismaticJoint.h>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

template<typename OneDofJoint>
Matrix4x4 getHomTransformDerivative(const OneDofJoint & joint, VectorDynSize& theta, const double step,
                                    const LinkIndex linkA, const LinkIndex linkB)
{
    Matrix4x4 ret;

    double originalTheta = theta(joint.getPosCoordsOffset());

    theta(joint.getPosCoordsOffset()) = originalTheta+step/2;
    Matrix4x4 perturbatedUpper = joint.getTransform(theta,linkA,linkB).asHomogeneousTransform();

    theta(joint.getPosCoordsOffset()) = originalTheta-step/2;
    Matrix4x4 perturbatedLower = joint.getTransform(theta,linkA,linkB).asHomogeneousTransform();

    theta(joint.getPosCoordsOffset()) = originalTheta;

    toEigen(ret) = (toEigen(perturbatedUpper)-toEigen(perturbatedLower))/step;

    return ret;
}

template<typename OneDofJoint>
Matrix6x6 getAdjTransformDerivative(const OneDofJoint & joint, VectorDynSize& theta,const double step,
                                    const LinkIndex linkA, const LinkIndex linkB)
{
    Matrix6x6 ret;

    double originalTheta = theta(joint.getPosCoordsOffset());

    theta(joint.getPosCoordsOffset()) = originalTheta+step/2;
    Matrix6x6 perturbatedUpper = joint.getTransform(theta,linkA,linkB).asAdjointTransform();

    theta(joint.getPosCoordsOffset()) = originalTheta-step/2;
    Matrix6x6 perturbatedLower = joint.getTransform(theta,linkA,linkB).asAdjointTransform();

    theta(joint.getPosCoordsOffset()) = originalTheta;

    toEigen(ret) = (toEigen(perturbatedUpper)-toEigen(perturbatedLower))/step;

    return ret;
}

template<typename OneDofJoint>
Matrix6x6 getAdjTransformWrenchDerivative(const OneDofJoint & joint, VectorDynSize& theta, const double step,
                                          const LinkIndex linkA, const LinkIndex linkB)
{
    Matrix6x6 ret;

    double originalTheta = theta(joint.getPosCoordsOffset());

    theta(joint.getPosCoordsOffset()) = originalTheta+step/2;
    Matrix6x6 perturbatedUpper = joint.getTransform(theta,linkA,linkB).asAdjointTransformWrench();

    theta(joint.getPosCoordsOffset()) = originalTheta-step/2;
    Matrix6x6 perturbatedLower = joint.getTransform(theta,linkA,linkB).asAdjointTransformWrench();

    theta(joint.getPosCoordsOffset()) = originalTheta;


    toEigen(ret) = (toEigen(perturbatedUpper)-toEigen(perturbatedLower))/step;

    return ret;
}

template<typename OneDofJoint>
void validateJointTransformDerivative(const OneDofJoint & joint, VectorDynSize& theta,
                                      const LinkIndex linkA, const LinkIndex linkB)
{
    double numericalDerivStep = 1e-8;
    double tol = numericalDerivStep*1e2;

    Transform           trans    = joint.getTransform(theta,linkA,linkB);
    TransformDerivative analytic = joint.getTransformDerivative(theta,linkA,linkB,1);

    Matrix4x4 homTransformDerivAn = analytic.asHomogeneousTransformDerivative();
    Matrix4x4 homTransformDerivNum = getHomTransformDerivative(joint,theta,numericalDerivStep,linkA,linkB);

    Matrix6x6 adjTransformDerivAn = analytic.asAdjointTransformDerivative(trans);
    Matrix6x6 adjTransformDerivNum = getAdjTransformDerivative(joint,theta,numericalDerivStep,linkA,linkB);

    Matrix6x6 adjWrenchTransformDerivAn = analytic.asAdjointTransformWrenchDerivative(trans);
    Matrix6x6 adjWrenchTransformDerivNum = getAdjTransformWrenchDerivative(joint,theta,numericalDerivStep,linkA,linkB);

    ASSERT_EQUAL_MATRIX_TOL(homTransformDerivAn,homTransformDerivNum,tol);
    ASSERT_EQUAL_MATRIX_TOL(adjTransformDerivAn,adjTransformDerivNum,tol);
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
template<typename OneDofJoint>
void validateJointMotionSubspaceMatrix(const OneDofJoint & joint, VectorDynSize& theta,
                                      const LinkIndex parent, const LinkIndex child)
{
    // Small time step for numerical differentiation
    const double dt = 1e-6;
    const double velocity = 1.0;  // Unit velocity for simplicity
    const double tol = 1e-5;

    // Get current transform from child to parent at position theta
    Transform P_H_C = joint.getTransform(theta, parent, child);
    Matrix4x4 P_H_C_matrix = P_H_C.asHomogeneousTransform();

    // Get the motion subspace vector
    SpatialMotionVector C_s_PC = joint.getMotionSubspaceVector(0, child, parent);

    // Calculate {}^P H_C * {}^C s * v (left side of the equation)
    // Convert the spatial motion vector to a 4x4 matrix using the wedge operator
    // Create a temporary Eigen vector to hold the spatial motion vector data
    Eigen::Matrix<double, 6, 1> spatialMotionEigen;
    spatialMotionEigen << toEigen(C_s_PC.getLinearVec3()), toEigen(C_s_PC.getAngularVec3());
    
    // Apply the wedge operator using our helper function
    Matrix4x4 C_s_wedge;
    toEigen(C_s_wedge) = wedge6dTo4x4d(spatialMotionEigen);

    // Calculate {}^P H_C * {}^C s * velocity
    Matrix4x4 left_side;
    toEigen(left_side) = toEigen(P_H_C_matrix) * toEigen(C_s_wedge) * velocity;

    // Now calculate the right side: the numerical time derivative of the transform
    Matrix4x4 right_side = getHomTransformDerivative(joint, theta, dt, parent, child);
    toEigen(right_side) = toEigen(right_side) * velocity;

    // Verify that {}^P H_C * {}^C s * velocity = {}^P \dot{H}_C
    ASSERT_EQUAL_MATRIX_TOL(left_side, right_side, tol);
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
        JointType joint(0, 1, getRandomTransform(), getRandomAxis());
        joint.setPosCoordsOffset(0);
        joint.setDOFsOffset(0);

        VectorDynSize jointPos(joint.getNrOfPosCoords());
        jointPos(joint.getPosCoordsOffset()) = getRandomDouble();

        // Test the joint transform derivatives in both directions
        validateJointTransformDerivative(joint, jointPos,
                                       joint.getFirstAttachedLink(), joint.getSecondAttachedLink());
        validateJointTransformDerivative(joint, jointPos,
                                       joint.getSecondAttachedLink(), joint.getFirstAttachedLink());

        // Test the relationship between transform and motion subspace in both directions
        validateJointMotionSubspaceMatrix(joint, jointPos,
                                        joint.getFirstAttachedLink(), joint.getSecondAttachedLink());
        validateJointMotionSubspaceMatrix(joint, jointPos,
                                        joint.getSecondAttachedLink(), joint.getFirstAttachedLink());

        if (printProgress) {
            std::cout << typeid(JointType).name() << " test " << i << " passed." << std::endl;
        }
    }
}

int main()
{
    // Test RevoluteJoint
    testJoint<RevoluteJoint>();
    
    // Test PrismaticJoint
    testJoint<PrismaticJoint>();
    
    std::cout << "All joint tests passed!" << std::endl;
    return EXIT_SUCCESS;
}
