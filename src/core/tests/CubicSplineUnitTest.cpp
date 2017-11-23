/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "iDynTree/Core/CubicSpline.h"
#include "iDynTree/Core/TestUtils.h"
#include "iDynTree/Core/VectorFixSize.h"
#include <cmath>

using namespace iDynTree;
using namespace std;

bool setNpoints(size_t n, double initialTime, double finalTime, const Vector4& parameters, CubicSpline& spline){

    assertTrue(n > 0);
    assertTrue(finalTime > initialTime);

    double dT = (finalTime-initialTime)/(static_cast<double>(n));
    VectorDynSize tVec, yVec;

    tVec.resize(n+1);
    yVec.resize(n+1);


    for (int i = 0; i <= n; ++i){
        tVec(i) = dT*i + initialTime;
        yVec(i) = parameters(0) + parameters(1)*tVec(i) + parameters(2)*pow(tVec(i),2) + parameters(3)*pow(tVec(i),3);
    }

    assertTrue(spline.setData(tVec, yVec));
    double initialVelocity = parameters(1) + 2*parameters(2)*initialTime + 3*parameters(3)*pow(initialTime,2);
    double initialAcceleration = 2*parameters(2) + 6*parameters(3)*initialTime;
    spline.setInitialConditions(initialVelocity, initialAcceleration);
    double finalVelocity = parameters(1) + 2*parameters(2)*finalTime + 3*parameters(3)*pow(finalTime,2);
    double finalAcceleration = 2*parameters(2) + 6*parameters(3)*finalTime;
    spline.setFinalConditions(finalVelocity, finalAcceleration);
    return true;
}

bool checkNpoints(size_t n, double initialTime, double finalTime, const Vector4& parameters, CubicSpline& spline, double relTol = DEFAULT_TOL){

    assertTrue(n > 0);
    assertTrue(finalTime > initialTime);

    double dT = (finalTime-initialTime)/(static_cast<double>(n));
    VectorDynSize tVec, yVec;
    double expected, t;


    for (int i =0; i <= n; ++i){
        t = dT*i + initialTime;
        expected = parameters(0) + parameters(1)*t + parameters(2)*pow(t,2) + parameters(3)*pow(t,3);
        assertDoubleAreEqual(expected, spline.evaluatePoint(t), std::abs(expected)*relTol);
    }
    return true;
}


bool splineTest(){
    srand(0);
    Vector4 parameters;

    for(size_t i = 0; i < 4; ++i)
        parameters(i) = getRandomDouble();

    CubicSpline spline;
    double initialTime = 1.0;
    double finalTime = 2.0;

    assertTrue(setNpoints(100,initialTime,finalTime,parameters, spline));
    assertTrue(checkNpoints(100, initialTime, finalTime, parameters, spline));
    assertTrue(checkNpoints(500, initialTime, finalTime, parameters, spline, 1E-2)); //the tolerance here is high, but there may be numeric errors while computing the coefficients for the spline, thus, for some points, the difference between the ideal cubic and the spline may be relevant. In any case, on the interpolation points they need to be equal.
    return true;
}

int main(){
    assertTrue(splineTest());
    return EXIT_SUCCESS;
}
