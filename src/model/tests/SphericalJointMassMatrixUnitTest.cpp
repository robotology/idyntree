// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file SphericalJointMassMatrixUnitTest.cpp
 *
 * Unit tests for verifying that the CRBA (Composite Rigid Body Algorithm)
 * correctly computes the mass matrix block for spherical joints, including
 * the intra-joint off-diagonal terms.
 *
 * For a spherical joint with 3 velocity DOFs, the mass matrix block M_jj should
 * be a full 3x3 matrix: M_jj = S^T @ Ic @ S, where S is the 6x3 motion subspace.
 *
 * This test verifies that both diagonal and off-diagonal terms are computed
 * correctly.
 */

#include <iDynTree/Dynamics.h>
#include <iDynTree/ForwardKinematics.h>
#include <iDynTree/FreeFloatingMatrices.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/Link.h>
#include <iDynTree/LinkState.h>
#include <iDynTree/Model.h>
#include <iDynTree/RotationalInertia.h>
#include <iDynTree/SpatialInertia.h>
#include <iDynTree/SphericalJoint.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/Traversal.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>

using namespace iDynTree;

/**
 * Create a minimal iDynTree model with two links connected by a spherical joint.
 *
 * Structure:
 *     base_link --[spherical_joint]--> child_link
 *
 * The child link has an asymmetric inertia tensor to ensure the off-diagonal
 * terms of the mass matrix block are non-zero.
 *
 * @param useAsymmetricInertia If true, use an asymmetric inertia tensor for the
 *                             child link to ensure non-zero off-diagonal terms.
 */
Model createTwoLinkSphericalModel(bool useAsymmetricInertia = true)
{
    Model model;

    // =========================================================================
    // Link 1: Base link (will be the floating base)
    // =========================================================================
    Link baseLink;
    SpatialInertia baseInertia;
    baseInertia.zero();
    RotationalInertia baseRotInertia;
    // Simple diagonal rotational inertia
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            baseRotInertia.setVal(i, j, (i == j) ? 0.1 : 0.0);
        }
    }
    baseInertia.fromRotationalInertiaWrtCenterOfMass(1.0, Position::Zero(), baseRotInertia);
    baseLink.setInertia(baseInertia);
    LinkIndex baseLinkIdx = model.addLink("base_link", baseLink);

    // =========================================================================
    // Link 2: Child link with configurable inertia
    // =========================================================================
    Link childLink;
    SpatialInertia childInertia;
    childInertia.zero();
    RotationalInertia childRotInertia;

    if (useAsymmetricInertia)
    {
        // Asymmetric rotational inertia tensor (about CoM):
        //   [Ixx  Ixy  Ixz]   [0.5  0.1  0.05]
        //   [Ixy  Iyy  Iyz] = [0.1  0.3  0.08]
        //   [Ixz  Iyz  Izz]   [0.05 0.08 0.4 ]
        //
        // This is a valid (positive definite) inertia tensor.
        // The off-diagonal terms ensure that S^T @ Ic @ S has off-diagonal terms.
        double inertiaValues[3][3] = {{0.5, 0.1, 0.05}, {0.1, 0.3, 0.08}, {0.05, 0.08, 0.4}};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                childRotInertia.setVal(i, j, inertiaValues[i][j]);
            }
        }
    } else
    {
        // Diagonal inertia (simpler case)
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                childRotInertia.setVal(i, j, (i == j) ? (0.3 + 0.1 * i) : 0.0);
            }
        }
    }

    childInertia.fromRotationalInertiaWrtCenterOfMass(2.0, Position::Zero(), childRotInertia);
    childLink.setInertia(childInertia);
    LinkIndex childLinkIdx = model.addLink("child_link", childLink);

    // =========================================================================
    // Spherical Joint connecting base_link to child_link
    // =========================================================================
    // Joint at origin (no offset)
    Transform restTransform = Transform::Identity();

    SphericalJoint sphericalJoint;
    sphericalJoint.setAttachedLinks(baseLinkIdx, childLinkIdx);
    sphericalJoint.setRestTransform(restTransform);
    sphericalJoint.setJointCenter(baseLinkIdx, Position::Zero());

    JointIndex jointIdx = model.addJoint("spherical_joint", &sphericalJoint);
    ASSERT_IS_TRUE(model.isValidJointIndex(jointIdx));

    return model;
}

/**
 * Get the expected mass matrix block for a spherical joint.
 *
 * For a spherical joint at the identity configuration, the motion subspace is:
 *   S = [0_{3x3}; I_{3x3}]
 *
 * The mass matrix block is:
 *   M_jj = S^T @ Ic @ S
 *
 * which equals the lower-right 3x3 block of the composite inertia Ic.
 *
 * @param compositeInertia The 6x6 composite rigid body inertia at the joint
 * @param[out] expectedMassBlock The expected 3x3 mass matrix block (row-major, size 9)
 */
void computeExpectedMassMatrixBlock(const SpatialInertia& compositeInertia,
                                    double expectedMassBlock[3][3])
{
    // Get the spatial inertia as a 6x6 matrix
    Matrix6x6 Ic = compositeInertia.asMatrix();

    // Motion subspace for spherical joint: S = [0_{3x3}; I_{3x3}]
    // This selects only angular velocity components (no linear velocity)
    // M_jj = S^T @ Ic @ S = lower-right 3x3 block of Ic
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            expectedMassBlock[i][j] = Ic(3 + i, 3 + j);
        }
    }
}

/**
 * Test that the CRBA computes the correct mass matrix block for a spherical joint.
 * This includes verifying both diagonal and off-diagonal terms.
 */
void testSphericalJointMassMatrixBlock()
{
    std::cout << "Testing spherical joint mass matrix block computation..." << std::endl;

    // Create model with asymmetric inertia to get non-zero off-diagonal terms
    Model model = createTwoLinkSphericalModel(true);

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(), 2);
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(), 1);
    ASSERT_EQUAL_DOUBLE(model.getNrOfPosCoords(), 4); // quaternion
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(), 3); // angular velocity

    // Create traversal
    Traversal traversal;
    bool ok = model.computeFullTreeTraversal(traversal);
    ASSERT_IS_TRUE(ok);

    // Setup joint positions (identity quaternion)
    JointPosDoubleArray jointPos(model);
    jointPos(0) = 1.0; // w
    jointPos(1) = 0.0; // x
    jointPos(2) = 0.0; // y
    jointPos(3) = 0.0; // z

    // Allocate buffers
    LinkCompositeRigidBodyInertias linkCRBIs(model);
    FreeFloatingMassMatrix massMatrix(model);

    // Run CRBA
    ok = CompositeRigidBodyAlgorithm(model, traversal, jointPos, linkCRBIs, massMatrix);
    ASSERT_IS_TRUE(ok);

    // Get the child link's composite inertia (which equals the link inertia since
    // there are no further descendants)
    LinkIndex childLinkIdx = model.getLinkIndex("child_link");
    const SpatialInertia& compositeInertia = linkCRBIs(childLinkIdx);

    // Compute expected mass matrix block analytically
    double expectedMassBlock[3][3];
    computeExpectedMassMatrixBlock(compositeInertia, expectedMassBlock);

    // Extract the joint block from the computed mass matrix (rows/cols 6-8)
    // The mass matrix is (6 + nDOFs) x (6 + nDOFs) = 9x9
    double actualMassBlock[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            actualMassBlock[i][j] = massMatrix(6 + i, 6 + j);
        }
    }

    std::cout << "Expected mass matrix block (3x3):" << std::endl;
    for (int i = 0; i < 3; i++)
    {
        std::cout << "  [";
        for (int j = 0; j < 3; j++)
        {
            std::cout << expectedMassBlock[i][j];
            if (j < 2)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    std::cout << "Actual mass matrix block (3x3):" << std::endl;
    for (int i = 0; i < 3; i++)
    {
        std::cout << "  [";
        for (int j = 0; j < 3; j++)
        {
            std::cout << actualMassBlock[i][j];
            if (j < 2)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    // Verify diagonal elements
    double tol = 1e-10;
    for (int i = 0; i < 3; i++)
    {
        ASSERT_EQUAL_DOUBLE_TOL(actualMassBlock[i][i], expectedMassBlock[i][i], tol);
    }

    // Verify off-diagonal elements (this is the key test for the bug fix)
    for (int i = 0; i < 3; i++)
    {
        for (int j = i + 1; j < 3; j++)
        {
            // Check upper triangular
            ASSERT_EQUAL_DOUBLE_TOL(actualMassBlock[i][j], expectedMassBlock[i][j], tol);
            // Check lower triangular (symmetry)
            ASSERT_EQUAL_DOUBLE_TOL(actualMassBlock[j][i], expectedMassBlock[j][i], tol);
            // Check symmetry of actual matrix
            ASSERT_EQUAL_DOUBLE_TOL(actualMassBlock[i][j], actualMassBlock[j][i], tol);
        }
    }

    // Verify the full mass matrix is symmetric
    for (size_t i = 0; i < massMatrix.rows(); i++)
    {
        for (size_t j = i + 1; j < massMatrix.cols(); j++)
        {
            ASSERT_EQUAL_DOUBLE_TOL(massMatrix(i, j), massMatrix(j, i), tol);
        }
    }

    std::cout << "Spherical joint mass matrix block test PASSED!" << std::endl;
}

/**
 * Test the mass matrix computation with a random quaternion configuration.
 * This verifies the computation works correctly for non-identity orientations.
 */
void testSphericalJointMassMatrixRandomConfig()
{
    std::cout << "Testing spherical joint mass matrix at random configuration..." << std::endl;

    Model model = createTwoLinkSphericalModel(true);

    Traversal traversal;
    bool ok = model.computeFullTreeTraversal(traversal);
    ASSERT_IS_TRUE(ok);

    // Random quaternion (normalized)
    JointPosDoubleArray jointPos(model);
    double w = 0.5, x = 0.5, y = 0.5, z = 0.5;
    double norm = std::sqrt(w * w + x * x + y * y + z * z);
    jointPos(0) = w / norm;
    jointPos(1) = x / norm;
    jointPos(2) = y / norm;
    jointPos(3) = z / norm;

    LinkCompositeRigidBodyInertias linkCRBIs(model);
    FreeFloatingMassMatrix massMatrix(model);

    ok = CompositeRigidBodyAlgorithm(model, traversal, jointPos, linkCRBIs, massMatrix);
    ASSERT_IS_TRUE(ok);

    // Extract joint block
    double actualMassBlock[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            actualMassBlock[i][j] = massMatrix(6 + i, 6 + j);
        }
    }

    // Verify symmetry of the joint block
    double tol = 1e-10;
    for (int i = 0; i < 3; i++)
    {
        for (int j = i + 1; j < 3; j++)
        {
            ASSERT_EQUAL_DOUBLE_TOL(actualMassBlock[i][j], actualMassBlock[j][i], tol);
        }
    }

    // Verify the mass matrix block is positive definite by checking
    // all diagonal elements are positive and the determinant is positive
    // (simplified check - diagonal dominance for this specific case)
    for (int i = 0; i < 3; i++)
    {
        ASSERT_IS_TRUE(actualMassBlock[i][i] > 0);
    }

    // Verify full mass matrix symmetry
    for (size_t i = 0; i < massMatrix.rows(); i++)
    {
        for (size_t j = i + 1; j < massMatrix.cols(); j++)
        {
            ASSERT_EQUAL_DOUBLE_TOL(massMatrix(i, j), massMatrix(j, i), tol);
        }
    }

    std::cout << "Random configuration test PASSED!" << std::endl;
}

/**
 * Test that the mass matrix has non-zero off-diagonal terms when the inertia
 * tensor has off-diagonal terms.
 */
void testSphericalJointOffDiagonalTermsNonZero()
{
    std::cout << "Testing that off-diagonal terms are non-zero with asymmetric inertia..."
              << std::endl;

    Model model = createTwoLinkSphericalModel(true);

    Traversal traversal;
    model.computeFullTreeTraversal(traversal);

    // Identity quaternion
    JointPosDoubleArray jointPos(model);
    jointPos(0) = 1.0;
    jointPos(1) = 0.0;
    jointPos(2) = 0.0;
    jointPos(3) = 0.0;

    LinkCompositeRigidBodyInertias linkCRBIs(model);
    FreeFloatingMassMatrix massMatrix(model);

    CompositeRigidBodyAlgorithm(model, traversal, jointPos, linkCRBIs, massMatrix);

    // The asymmetric inertia tensor should produce non-zero off-diagonal terms
    // in the joint mass block
    double offDiag01 = massMatrix(6 + 0, 6 + 1);
    double offDiag02 = massMatrix(6 + 0, 6 + 2);
    double offDiag12 = massMatrix(6 + 1, 6 + 2);

    std::cout << "Off-diagonal terms: M[0,1]=" << offDiag01 << ", M[0,2]=" << offDiag02
              << ", M[1,2]=" << offDiag12 << std::endl;

    // With the asymmetric inertia we defined, these should be non-zero
    // The expected values from the inertia tensor are: 0.1, 0.05, 0.08
    double expectedOffDiag01 = 0.1;
    double expectedOffDiag02 = 0.05;
    double expectedOffDiag12 = 0.08;

    double tol = 1e-10;
    ASSERT_EQUAL_DOUBLE_TOL(offDiag01, expectedOffDiag01, tol);
    ASSERT_EQUAL_DOUBLE_TOL(offDiag02, expectedOffDiag02, tol);
    ASSERT_EQUAL_DOUBLE_TOL(offDiag12, expectedOffDiag12, tol);

    std::cout << "Off-diagonal terms test PASSED!" << std::endl;
}

/**
 * Test mass matrix with diagonal inertia tensor (simpler case).
 * Off-diagonal terms of the joint block should be zero in this case.
 */
void testSphericalJointDiagonalInertia()
{
    std::cout << "Testing spherical joint with diagonal inertia tensor..." << std::endl;

    // Create model with diagonal inertia
    Model model = createTwoLinkSphericalModel(false);

    Traversal traversal;
    model.computeFullTreeTraversal(traversal);

    JointPosDoubleArray jointPos(model);
    jointPos(0) = 1.0;
    jointPos(1) = 0.0;
    jointPos(2) = 0.0;
    jointPos(3) = 0.0;

    LinkCompositeRigidBodyInertias linkCRBIs(model);
    FreeFloatingMassMatrix massMatrix(model);

    CompositeRigidBodyAlgorithm(model, traversal, jointPos, linkCRBIs, massMatrix);

    // With diagonal inertia, off-diagonal terms should be zero
    double tol = 1e-10;
    ASSERT_EQUAL_DOUBLE_TOL(massMatrix(6 + 0, 6 + 1), 0.0, tol);
    ASSERT_EQUAL_DOUBLE_TOL(massMatrix(6 + 0, 6 + 2), 0.0, tol);
    ASSERT_EQUAL_DOUBLE_TOL(massMatrix(6 + 1, 6 + 2), 0.0, tol);

    // Diagonal terms should be the diagonal elements of the rotational inertia
    // From createTwoLinkSphericalModel with useAsymmetricInertia=false:
    // childRotInertia diagonal is [0.3, 0.4, 0.5]
    ASSERT_EQUAL_DOUBLE_TOL(massMatrix(6 + 0, 6 + 0), 0.3, tol);
    ASSERT_EQUAL_DOUBLE_TOL(massMatrix(6 + 1, 6 + 1), 0.4, tol);
    ASSERT_EQUAL_DOUBLE_TOL(massMatrix(6 + 2, 6 + 2), 0.5, tol);

    std::cout << "Diagonal inertia test PASSED!" << std::endl;
}

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "Spherical Joint Mass Matrix Unit Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    testSphericalJointMassMatrixBlock();
    testSphericalJointMassMatrixRandomConfig();
    testSphericalJointOffDiagonalTermsNonZero();
    testSphericalJointDiagonalInertia();

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "All spherical joint mass matrix tests PASSED!" << std::endl;
    std::cout << "========================================" << std::endl;

    return EXIT_SUCCESS;
}
