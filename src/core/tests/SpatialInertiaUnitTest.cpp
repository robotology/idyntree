/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/SpatialAcc.h>

#include <Eigen/Dense>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

void checkInertiaTransformation(const Transform & trans, const SpatialInertia & inertia)
{
    SpatialInertia inertiaTranslated = trans*inertia;
    Matrix6x6      inertiaTranslatedCheck;

    Matrix6x6 adjWre = trans.asAdjointTransformWrench();
    Matrix6x6 I      = inertia.asMatrix();
    Matrix6x6 adj    = trans.inverse().asAdjointTransform();
    toEigen(inertiaTranslatedCheck) = toEigen(adjWre)*
                                      toEigen(I)*
                                      toEigen(adj);

    Matrix6x6 inertiaTranslatedRaw = inertiaTranslated.asMatrix();
    ASSERT_EQUAL_MATRIX(inertiaTranslatedCheck,inertiaTranslatedRaw);
}

void checkInertiaTwistProduct(const SpatialInertia & inertia, const Twist & twist)
{
    SpatialMomentum momentum = inertia*twist;
    Vector6         momentumCheck;

    Matrix6x6 I = inertia.asMatrix();

    Vector6 twistPlain = twist.asVector();
    toEigen(momentumCheck) = toEigen(I)*toEigen(twistPlain);

    ASSERT_EQUAL_VECTOR(momentum.asVector(),momentumCheck);
}

void checkInvariance(const Transform & trans, const SpatialInertia & inertia, const Twist & twist)
{
    Transform invTrans = trans.inverse();
    SpatialMomentum momentumTranslated = trans*(inertia*twist);
    SpatialMomentum momentumTranslatedCheck = (trans*inertia)*(trans*twist);

    Twist           twistTranslated         = trans*twist;
    SpatialInertia  inertiaTranslated       = trans*inertia;
    Vector6 momentumTranslatedCheck2;
    Vector6 momentumTranslatedCheck3;
    Vector6 twistTranslatedCheck;
    Matrix6x6 transAdjWrench = trans.asAdjointTransformWrench();
    Matrix6x6 inertiaRaw     = inertia.asMatrix();
    Matrix6x6 transInvAdj    = trans.inverse().asAdjointTransform();
    Matrix6x6 transAdj       = trans.asAdjointTransform();
    Matrix6x6 inertiaTranslatedCheck;
    Vector6 twistPlain = twist.asVector();

    toEigen(momentumTranslatedCheck2) = toEigen(transAdjWrench)*
                                        toEigen(inertiaRaw)*
                                        toEigen(twistPlain);

    toEigen(momentumTranslatedCheck3) = toEigen(transAdjWrench)*
                                        toEigen(inertiaRaw)*
                                        toEigen(transInvAdj)*
                                        toEigen(transAdj)*
                                        toEigen(twistPlain);

    toEigen(twistTranslatedCheck)     = toEigen(transAdj)*
                                        toEigen(twistPlain);

    toEigen(inertiaTranslatedCheck)   = toEigen(transAdjWrench)*
                                        toEigen(inertiaRaw)*
                                        toEigen(transInvAdj);

    ASSERT_EQUAL_MATRIX(inertiaTranslatedCheck,inertiaTranslated.asMatrix());
    ASSERT_EQUAL_VECTOR(twistTranslated.asVector(),twistTranslatedCheck);
    ASSERT_EQUAL_VECTOR(momentumTranslated.asVector(),momentumTranslatedCheck3);
    ASSERT_EQUAL_VECTOR(momentumTranslated.asVector(),momentumTranslatedCheck2);
    ASSERT_EQUAL_VECTOR(momentumTranslated.asVector(),momentumTranslatedCheck.asVector());

    SpatialMomentum momentum = invTrans*momentumTranslated;
    SpatialMomentum momentumCheck = (invTrans*(trans*inertia))*(invTrans*(trans*twist));

    ASSERT_EQUAL_VECTOR(momentum.asVector(),momentumCheck.asVector());
}

void checkBiasWrench(const SpatialInertia & inertia, const Twist & twist)
{
    Wrench biasWrench = inertia.biasWrench(twist);
    Wrench biasWrenchCheck = twist*(inertia*twist);

    ASSERT_EQUAL_VECTOR(biasWrench.asVector(),biasWrenchCheck.asVector());
}

void checkRegressors(const SpatialInertia & I,
                     const Twist & v,
                     const SpatialAcc & a,
                     const Twist & vRef)
{
    //////////
    /// Check the momentum regressor
    /////////
    // Compute momentum with classical operation
    SpatialMomentum h = I*v;

    // Compute the inertial parameters for the given spatial inertia
    Vector10 inertialParams = I.asVector();

    // Compute momentum with regressors
    Vector6 hRegr;

    toEigen(hRegr) = toEigen(SpatialInertia::momentumRegressor(v))*toEigen(inertialParams);

    // Check that they are equal
    ASSERT_EQUAL_VECTOR(h.asVector(),hRegr);

    //////////
    /// Check the momentum derivative (net wrench) regressor
    /////////

    // Compute momentum derivative with classical operation
    Wrench dotH = I*a+v*(I*v);

    // Compute momentum derivative with regressors
    Vector6 dotHregr;

    toEigen(dotHregr) = toEigen(SpatialInertia::momentumDerivativeRegressor(v,a))*toEigen(inertialParams);

    // Check that they are equal
    ASSERT_EQUAL_VECTOR(dotH.asVector(),dotHregr);

    //////////
    /// Check the momentum derivative (net wrench) regressor, Slotine Li version
    /// Check also that the operator is simmetric with respect to v, vDes
    /////////

    Wrench dotH_SL = I*a+v*(I*vRef)-I*(v*vRef);

    Vector6 dotH_SL_regr;

    toEigen(dotH_SL_regr) =
        toEigen(SpatialInertia::momentumDerivativeSlotineLiRegressor(v,vRef,a))*toEigen(inertialParams);

    ASSERT_EQUAL_VECTOR(dotH_SL.asVector(),dotH_SL_regr);

    // Check that the regressor matches the momentumDerivativeRegressor one if v = vRef
    Wrench dotH_SL_check = I*a+v*(I*v)-I*(v*v);

    Vector6 dotH_SL_regr_check;

    toEigen(dotH_SL_regr_check) =
        toEigen(SpatialInertia::momentumDerivativeSlotineLiRegressor(v,v,a))*toEigen(inertialParams);

    ASSERT_EQUAL_VECTOR(dotH_SL_check.asVector(),dotH_SL_regr_check);
    ASSERT_EQUAL_VECTOR(dotH.asVector(),dotH_SL_check.asVector());

}

int main()
{
    Transform trans(Rotation::RPY(5.0,6.0,-4.0),Position(10,6,-4));

    double twistData[6] = {-5.0,-6.0,-5.0,1.0,2.0,3.0};
    Twist twist(LinVelocity(twistData,3),AngVelocity(twistData+3,3));

    ASSERT_EQUAL_DOUBLE(twist.asVector()(0),twistData[0]);

    double rotInertiaData[3*3] = {10.0,0.04,0.04,
                                  0.04,20.0,0.04,
                                  0.04,0.04,24.0};
    SpatialInertia inertia(1.0,Position(100,-5,10),RotationalInertiaRaw(rotInertiaData,3,3));

    checkInertiaTwistProduct(inertia,twist);
    checkInertiaTransformation(trans,inertia);
    checkInvariance(trans,inertia,twist);
    checkBiasWrench(inertia,twist);

    int nrOfChecks = 30;
    for(int i=0; i < nrOfChecks; i++ )
    {
        inertia = getRandomInertia();
        twist   = getRandomTwist();
        Twist twist2  = getRandomTwist();
        SpatialAcc a  = getRandomTwist();

        checkRegressors(inertia,twist,a,twist2);
    }

    return EXIT_SUCCESS;
}