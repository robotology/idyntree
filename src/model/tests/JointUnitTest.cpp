// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Link.h>

#include <iDynTree/Position.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/Transform.h>
#include <iDynTree/TransformDerivative.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/EigenHelpers.h>

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

int main()
{
    for(unsigned int i=0; i < 10; i++)
    {
        // Random revolute joint
        // connecting links 0 and 1
        RevoluteJoint revJoint(0,1,getRandomTransform(),getRandomAxis());
        revJoint.setPosCoordsOffset(0);
        revJoint.setDOFsOffset(0);

        VectorDynSize jointPos(revJoint.getNrOfPosCoords());
        jointPos(revJoint.getPosCoordsOffset()) = getRandomDouble();

        // Test the joint in both directions
        validateJointTransformDerivative(revJoint,jointPos,
                                         revJoint.getFirstAttachedLink(),revJoint.getSecondAttachedLink());
        validateJointTransformDerivative(revJoint,jointPos,
                                         revJoint.getSecondAttachedLink(),revJoint.getFirstAttachedLink());
    }
    
    for(unsigned int i=0; i < 10; i++)
    {
        // Random prismatic joint
        // connecting links 0 and 1
        PrismaticJoint priJoint(0, 1, getRandomTransform(), getRandomAxis());
        priJoint.setPosCoordsOffset(0);
        priJoint.setDOFsOffset(0);

        VectorDynSize jointPos(priJoint.getNrOfPosCoords());
        jointPos(priJoint.getPosCoordsOffset()) = getRandomDouble();

        // Test the joint in both directions
        validateJointTransformDerivative(priJoint, jointPos,
                                         priJoint.getFirstAttachedLink(), priJoint.getSecondAttachedLink());
        validateJointTransformDerivative(priJoint, jointPos,
                                         priJoint.getSecondAttachedLink(), priJoint.getFirstAttachedLink());
    }

    return EXIT_SUCCESS;
}
