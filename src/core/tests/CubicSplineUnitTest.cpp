// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "iDynTree/CubicSpline.h"
#include "iDynTree/TestUtils.h"
#include "iDynTree/VectorFixSize.h"
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

    double initialVelocity = parameters(1) + 2*parameters(2)*initialTime + 3*parameters(3)*pow(initialTime,2);
    double initialAcceleration = 2*parameters(2) + 6*parameters(3)*initialTime;
    spline.setInitialConditions(initialVelocity, initialAcceleration);
    double finalVelocity = parameters(1) + 2*parameters(2)*finalTime + 3*parameters(3)*pow(finalTime,2);
    double finalAcceleration = 2*parameters(2) + 6*parameters(3)*finalTime;
    spline.setFinalConditions(finalVelocity, finalAcceleration);

    assertTrue(spline.setData(tVec, yVec));
    return true;
}

bool checkNpoints(size_t n, double initialTime, double finalTime, const Vector4& parameters, CubicSpline& spline, double relTol = DEFAULT_TOL){

    assertTrue(n > 0);
    assertTrue(finalTime > initialTime);

    double dT = (finalTime-initialTime)/(static_cast<double>(n));
    VectorDynSize tVec, yVec;
    double expected, t;
    double velocity, acceleration;

    for (int i =0; i <= n; ++i){
        t = dT*i + initialTime;
        expected = parameters(0) + parameters(1)*t + parameters(2)*pow(t,2) + parameters(3)*pow(t,3);
        assertDoubleAreEqual(expected, spline.evaluatePoint(t, velocity, acceleration), std::abs(expected)*relTol, "Pos i = ", i);
        expected = parameters(1) + 2*parameters(2)*t + 3*parameters(3)*pow(t,2);
        assertDoubleAreEqual(expected, velocity, std::abs(expected)*relTol, "Vel i = ", i);
        expected = 2*parameters(2) + 6*parameters(3)*t;
        assertDoubleAreEqual(expected, acceleration, std::abs(expected)*relTol, "Acc i = ", i);
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
    assertTrue(checkNpoints(500, initialTime, finalTime, parameters, spline));
    return true;
}

int main(){
    assertTrue(splineTest());
    return EXIT_SUCCESS;
}
