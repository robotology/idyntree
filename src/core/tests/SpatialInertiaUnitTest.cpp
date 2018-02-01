/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
#include <iDynTree/Core/InertiaNonLinearParametrization.h>

#include <Eigen/Dense>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

SpatialInertia getNonPhysicalConsistentInertia()
{
    Vector16 paramsVec;
    paramsVec.zero();
    paramsVec(0) = -1.0;

    paramsVec(1) = 1.0;
    paramsVec(2) = 1.0;
    paramsVec(3) = 0.0;
    /*
    paramsVec(4) = 0.66474;
    paramsVec(5) = 0.534487;
    paramsVec(6) = -0.521962;
    paramsVec(7) = 0.124066;
    paramsVec(8) = 0.609993;
    paramsVec(9) = 0.782634;
    paramsVec(10) = 0.736701;
    paramsVec(11) = -0.585006;
    paramsVec(12) = 0.339175;
    paramsVec(13) = 0.434594;
    paramsVec(14) = -0.716795;
    paramsVec(15) = 0.213938;*/
    RigidBodyInertiaNonLinearParametrization params;
    params.fromVectorWithRotationAsVec(paramsVec);
    SpatialInertia I = params.toRigidBodyInertia();
    return I;
}

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


    std::cerr << "inertiaTranslated :\n" << inertiaTranslated.asMatrix().toString() << std::endl;
    std::cerr << "inertiaTranslatedCheck :\n" << inertiaTranslatedCheck.toString() << std::endl;

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

// Check the nonlinear parametrization of inertia
void checkInertiaNonLinearParametrization()
{
    RigidBodyInertiaNonLinearParametrization inertiaParams, inertiaParamsNotConsistent, inertiaParamsCheck;
    // populate the inertia parametrization with random (but physical consistent data)
    inertiaParams.mass = getRandomDouble(0.3,10.0);
    inertiaParams.com = getRandomPosition();
    inertiaParams.link_R_centroidal = getRandomRotation();
    inertiaParams.centralSecondMomentOfMass(2) = getRandomDouble(0.0,10.0);
    inertiaParams.centralSecondMomentOfMass(1) = getRandomDouble(0.0,inertiaParams.centralSecondMomentOfMass(2));
    inertiaParams.centralSecondMomentOfMass(0) = getRandomDouble(0.0,inertiaParams.centralSecondMomentOfMass(1));

    ASSERT_IS_TRUE(inertiaParams.isPhysicallyConsistent());

    inertiaParamsNotConsistent.mass = getRandomDouble(-10.0,-0.3);
    inertiaParamsNotConsistent.com.zero();
    inertiaParamsNotConsistent.link_R_centroidal =  getRandomRotation(); //iDynTree::Rotation::Identity();
    inertiaParamsNotConsistent.centralSecondMomentOfMass(0) = 1.0; //getRandomDouble(0.0,10.0);
    inertiaParamsNotConsistent.centralSecondMomentOfMass(1) = 2.0; //getRandomDouble(0.0,inertiaParams.centralSecondMomentOfMass(0));
    inertiaParamsNotConsistent.centralSecondMomentOfMass(2) = 3.0; //getRandomDouble(0.0,inertiaParams.centralSecondMomentOfMass(1));

    ASSERT_IS_FALSE(inertiaParamsNotConsistent.isPhysicallyConsistent());

    // Check if the two nonlinear-->inertia--> nonlinear works fine
    inertiaParamsCheck.fromRigidBodyInertia(inertiaParams.toRigidBodyInertia());

    ASSERT_EQUAL_DOUBLE(inertiaParams.mass,inertiaParamsCheck.mass);
    ASSERT_EQUAL_VECTOR(inertiaParams.com,inertiaParamsCheck.com);

    //ASSERT_EQUAL_MATRIX(inertiaParams.link_R_centroidal,inertiaParamsCheck.link_R_centroidal);

    ASSERT_EQUAL_VECTOR(inertiaParams.centralSecondMomentOfMass,inertiaParamsCheck.centralSecondMomentOfMass);
    ASSERT_EQUAL_MATRIX(inertiaParams.toRigidBodyInertia().asMatrix(),inertiaParamsCheck.toRigidBodyInertia().asMatrix());

    // Check mapping to and from usual inertial parameters
    RigidBodyInertiaNonLinearParametrization inertiaParamsCheck2;

    inertiaParamsCheck2.fromInertialParameters(inertiaParams.toRigidBodyInertia().asVector());

    ASSERT_EQUAL_MATRIX(inertiaParams.toRigidBodyInertia().asMatrix(),inertiaParamsCheck2.toRigidBodyInertia().asMatrix());
}

void checkInertiaNonLinearParametrizationGradient()
{
    RigidBodyInertiaNonLinearParametrization inertiaParams;

    inertiaParams.mass = getRandomDouble(0.3,10.0);
    inertiaParams.com = getRandomPosition();
    inertiaParams.link_R_centroidal =  getRandomRotation();
    inertiaParams.centralSecondMomentOfMass(2) = getRandomDouble(0.0,10.0);
    inertiaParams.centralSecondMomentOfMass(1) = getRandomDouble(0.0,inertiaParams.centralSecondMomentOfMass(2));
    inertiaParams.centralSecondMomentOfMass(0) = getRandomDouble(0.0,inertiaParams.centralSecondMomentOfMass(1));

    // Analytical gradient
    Matrix10x16 gradient = inertiaParams.getGradientWithRotationAsVec();

    // Numerical gradient
    Matrix10x16 numericalGradient;

    // Let's compute the numericalGradient
    double delta = 1e-4;
    Vector16 currentValue = inertiaParams.asVectorWithRotationAsVec();
    Vector10 currentOutput = inertiaParams.toRigidBodyInertia().asVector();

    Vector16 upperPerturbation;
    Vector16 lowerPerturbation;
    Vector10 upperOutput;
    Vector10 lowerOutput;
    for(int i=0; i < 16; i++ )
    {
        upperPerturbation = currentValue;
        lowerPerturbation = currentValue;

        upperPerturbation(i) = currentValue(i) + delta;
        lowerPerturbation(i) = currentValue(i) - delta;

        RigidBodyInertiaNonLinearParametrization upperPertubed,lowerPertubed;
        upperPertubed.fromVectorWithRotationAsVec(upperPerturbation);
        lowerPertubed.fromVectorWithRotationAsVec(lowerPerturbation);

        upperOutput = upperPertubed.toRigidBodyInertia().asVector();
        lowerOutput = lowerPertubed.toRigidBodyInertia().asVector();


        for(int j = 0; j < 10; j++)
        {
            numericalGradient(j,i) = (upperOutput(j)-lowerOutput(j))/(2*delta);
        }
    }

    // Check that the two are consistent
    ASSERT_EQUAL_MATRIX_TOL(gradient,numericalGradient,1e-8);

}

void checkRegressorsAsGradients()
{
    SpatialAcc a = getRandomTwist();
    Twist v = getRandomTwist();
    SpatialInertia I = getRandomInertia();

    Matrix6x10 momentumGradient, momentumDerivativeGradient;
    Matrix6x10 momentumGradientNum, momentumDerivativeGradientNum;

    momentumGradient = SpatialInertia::momentumRegressor(v);
    momentumDerivativeGradient = SpatialInertia::momentumDerivativeRegressor(v,a);

    double delta = 1e-4;
    Vector10 currentValue = I.asVector();
    Vector6  currentOutputMom = (I*v).asVector();
    Vector6  currentOutputMomDer = (I*a+v.cross(I*v)).asVector();

    Vector10 upperPerturbation;
    Vector10 lowerPerturbation;
    Vector6 upperOutputMom, lowerOutputMom, upperOutputMomDer, lowerOutputMomDer;
    for(int i=0; i < 10; i++ )
    {
        upperPerturbation = currentValue;
        lowerPerturbation = currentValue;

        upperPerturbation(i) = currentValue(i) + delta;
        lowerPerturbation(i) = currentValue(i) - delta;

        SpatialInertia upperPertubed,lowerPertubed;
        upperPertubed.fromVector(upperPerturbation);
        lowerPertubed.fromVector(lowerPerturbation);

        upperOutputMom = (upperPertubed*v).asVector();
        lowerOutputMom = (lowerPertubed*v).asVector();

        upperOutputMomDer = (upperPertubed*a+v.cross(upperPertubed*v)).asVector();
        lowerOutputMomDer = (lowerPertubed*a+v.cross(lowerPertubed*v)).asVector();

        for(int j = 0; j < 6; j++)
        {
            momentumGradientNum(j,i) = (upperOutputMom(j)-lowerOutputMom(j))/(2*delta);
            momentumDerivativeGradientNum(j,i) =  (upperOutputMomDer(j)-lowerOutputMomDer(j))/(2*delta);
        }
    }

    // Check that the two are consistent
    ASSERT_EQUAL_MATRIX_TOL(momentumGradient,momentumGradientNum,1e-8);
    ASSERT_EQUAL_MATRIX_TOL(momentumDerivativeGradient,momentumDerivativeGradientNum,1e-8);

}

void checkInertiaIdentificationCostFunction()
{
    SpatialAcc a;
    a.zero();
    a(3) = 1.0;
    Twist v;
    v.zero();
    Vector16 paramsVec;
    paramsVec.zero();
    paramsVec(0) = -1.0;

    paramsVec(1) = 1.0;
    paramsVec(2) = 1.0;
    paramsVec(3) = 0.0;
    /*
    paramsVec(4) = 0.66474;
    paramsVec(5) = 0.534487;
    paramsVec(6) = -0.521962;
    paramsVec(7) = 0.124066;
    paramsVec(8) = 0.609993;
    paramsVec(9) = 0.782634;
    paramsVec(10) = 0.736701;
    paramsVec(11) = -0.585006;
    paramsVec(12) = 0.339175;
    paramsVec(13) = 0.434594;
    paramsVec(14) = -0.716795;
    paramsVec(15) = 0.213938;*/
    RigidBodyInertiaNonLinearParametrization params;
    params.fromVectorWithRotationAsVec(paramsVec);
    SpatialInertia I = params.toRigidBodyInertia();

    Wrench f;
    f.zero();
    f(3) = 1.0;

    Vector6 residual = (f-I*a).asVector();
    double cost = toEigen(residual).transpose()*toEigen(residual);
    Vector16 costGradient;
    toEigen(costGradient) = 2*toEigen(I*a-f).transpose()*toEigen(SpatialInertia::momentumDerivativeRegressor(v,a))*toEigen(params.getGradientWithRotationAsVec());

    double delta = 1e-4;
    Vector16 currentValue = paramsVec;
    double currentOutput = cost;

    Vector16 upperPerturbation;
    Vector16 lowerPerturbation;
    Vector16 costGradientNumerical;
    for(int i=0; i < 16; i++ )
    {
        upperPerturbation = currentValue;
        lowerPerturbation = currentValue;

        upperPerturbation(i) = currentValue(i) + delta;
        lowerPerturbation(i) = currentValue(i) - delta;

        SpatialInertia upperPertubed,lowerPertubed;
        RigidBodyInertiaNonLinearParametrization upperParams, lowerParams;
        upperParams.fromVectorWithRotationAsVec(upperPerturbation);
        lowerParams.fromVectorWithRotationAsVec(lowerPerturbation);
        upperPertubed = upperParams.toRigidBodyInertia();
        lowerPertubed = lowerParams.toRigidBodyInertia();

        Vector6 upperResidual = (f-upperPertubed*a).asVector();
        double upperOutput = toEigen(upperResidual).transpose()*toEigen(upperResidual);

        Vector6 lowerResidual = (f-lowerPertubed*a).asVector();
        double lowerOutput = toEigen(lowerResidual).transpose()*toEigen(lowerResidual);

        costGradientNumerical(i) = (upperOutput-lowerOutput)/(2*delta);
    }

    /*
    std::cerr << "Residual                " << residual.toString() << std::endl;
    std::cerr << "I                     \n" << I.asMatrix().toString() << std::endl;
    std::cerr << "Cost                    " << cost << std::endl;
    std::cerr << "Cost gradient           " << costGradient.toString() << std::endl;
    std::cerr << "Cost gradient numerical " << costGradientNumerical.toString() << std::endl;
    std::cerr << "toEigen(SpatialInertia::momentumDerivativeRegressor(v,a)):\n" << SpatialInertia::momentumDerivativeRegressor(v,a).toString() << std::endl;
    std::cerr << "toEigen(params.getGradientWithRotationAsVec()):\n" << (params.getGradientWithRotationAsVec()).toString() << std::endl;
    */

    // Check that the two are consistent
    ASSERT_EQUAL_VECTOR_TOL(costGradient,costGradientNumerical,1e-6);

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

    inertia = getNonPhysicalConsistentInertia();

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

        checkInertiaNonLinearParametrization();
        checkInertiaNonLinearParametrizationGradient();
        checkRegressorsAsGradients();
    }

    checkInertiaIdentificationCostFunction();

    return EXIT_SUCCESS;
}