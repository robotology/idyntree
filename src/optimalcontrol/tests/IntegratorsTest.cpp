/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Integrator.h>
#include <iDynTree/Integrators/RK4.h>
#include <iDynTree/Integrators/ForwardEuler.h>
#include <iDynTree/Controller.h>
#include <memory>
#include <cmath>
#include <iostream>
#include <ctime>

using namespace iDynTree;
using namespace iDynTree::optimalcontrol;
using namespace iDynTree::optimalcontrol::integrators;


class TestEquation1 : public DynamicalSystem{
    double m_lambda;
    VectorDynSize m_initialConditions;
public:
    TestEquation1(double lambda)
        : DynamicalSystem(1,0)
        , m_lambda(lambda)
        , m_initialConditions(1)
    {}

    bool dynamics(const VectorDynSize &state, double time, VectorDynSize &stateDynamics){
        if (state.size() != 1) {
            return false;
        }
        stateDynamics.resize(1);
        stateDynamics(0) = m_lambda * state(0);
        return true;
    }

    void setInitialCondition(double x){
        m_initialConditions(0) = x;
    }

    const VectorDynSize& initialState() const{
        return m_initialConditions;
    }

    bool  dynamicsStateFirstDerivative(const VectorDynSize& state,
                                                        double time,
                                                        MatrixDynSize& dynamicsDerivative) override
    {
        dynamicsDerivative.resize(1,1);
        dynamicsDerivative(0,0) =m_lambda;
        return true; }

    bool dynamicsControlFirstDerivative(const VectorDynSize& state,
                                                         double time,
                                                         MatrixDynSize& dynamicsDerivative) override
    {
        dynamicsDerivative.resize(0,0);
        return true; }

};

class TestEquation2 : public DynamicalSystem{
    double m_lambda1, m_lambda2;
    VectorDynSize m_initialConditions;
public:
    TestEquation2(double lambda1, double lambda2)
        :DynamicalSystem(2,0)
        ,m_lambda1(lambda1)
        ,m_lambda2(lambda2)
        ,m_initialConditions(2)
    {}

    bool dynamics(const VectorDynSize &state, double time, VectorDynSize &stateDynamics){
        if (state.size() != 2)
            return false;
        stateDynamics.resize(2);
        stateDynamics(0) = m_lambda1 * state(0);
        stateDynamics(1) = m_lambda2 * state(1);
        return true;
    }

    void setInitialCondition(double x1, double x2){
        m_initialConditions(0) = x1;
        m_initialConditions(1) = x2;
    }

    const VectorDynSize& initialState() const{
        return m_initialConditions;
    }

};

class TestController : public Controller{
    double m_lambda;
    double m_state;
public:
    TestController(double lambda)
        :Controller(1)
        ,m_lambda(lambda)
    {}

    bool doControl(VectorDynSize &controllerOutput){
        controllerOutput.resize(1);
        controllerOutput(0) = m_lambda * m_state;
        return true;
    }

    bool setStateFeedback(const double t, const VectorDynSize &stateFeedback){
        if (stateFeedback.size() != 1)
            return false;

        m_state = stateFeedback(0);
        return true;
    }
};

class ControlledTestEquation : public DynamicalSystem{
    VectorDynSize m_initialConditions, m_controllerOutput;
    TestController m_controller;
public:
    ControlledTestEquation(double lambda)
        :DynamicalSystem(1,1)
        ,m_initialConditions(1)
        ,m_controllerOutput(1)
        ,m_controller(lambda)
    {}

    bool dynamics(const VectorDynSize &state, double time, VectorDynSize &stateDynamics){
        if (state.size() != 1)
            return false;

        if(!m_controller.setStateFeedback(time, state))
            return false;

        if(!m_controller.doControl(m_controllerOutput))
            return false;

        stateDynamics(0) = m_controllerOutput(0);
        return true;
    }

    void setInitialCondition(double x){
        m_initialConditions(0) = x;
    }

    const VectorDynSize& initialState() const{
        return m_initialConditions;
    }

};

double lambda1 = 1;
double lambda2 = 0.01;
double dT = 0.005;
double x1 =-1.0;
double x2 =2.0;
double initTime = 1.0;
double endTime = 20.0;
double relTol = 1E-8;

void IntegratorTest1(Integrator &toBeTested) {

    ASSERT_IS_TRUE(toBeTested.setMaximumStepSize(dT));
    ASSERT_IS_TRUE(toBeTested.integrate(initTime, endTime));

    int iterations = std::round((endTime-initTime)/dT);
    double t = initTime;
    iDynTree::VectorDynSize sol;
    double expected;

    for (int i = 0; i <= iterations; ++i){
        t = initTime + dT*i;
        ASSERT_IS_TRUE(toBeTested.getSolution(t, sol));
        expected = x1 * std::exp(lambda1*(t - initTime));
        ASSERT_EQUAL_DOUBLE_TOL(expected, sol(0), std::abs(expected)*relTol); //up to the eight significative digit
    }

}

void IntegratorTest2(Integrator &toBeTested) {
    ASSERT_IS_TRUE(toBeTested.setMaximumStepSize(dT));
    ASSERT_IS_TRUE(toBeTested.integrate(initTime, endTime));

    int iterations = std::round((endTime-initTime)/dT);
    double t = initTime;
    iDynTree::VectorDynSize sol;
    double expected;

    for (int i = 0; i <= iterations; ++i){
        t = initTime + dT*i;
        ASSERT_IS_TRUE(toBeTested.getSolution(t, sol));
        expected = x1 * std::exp(lambda1*(t - initTime));
        ASSERT_EQUAL_DOUBLE_TOL(expected, sol(0), std::abs(expected)*relTol); //up to the eight significative digit
        expected = x2 * std::exp(lambda2*(t - initTime));
        ASSERT_EQUAL_DOUBLE_TOL(expected, sol(1), std::abs(expected)*relTol); //up to the eight significative digit
    }
}

void IntegratorTest3(Integrator &toBeTested) {

    ASSERT_IS_TRUE(toBeTested.setMaximumStepSize(dT));
    ASSERT_IS_TRUE(toBeTested.integrate(initTime, endTime));

    int iterations = std::round((endTime-initTime)/dT);
    double t = initTime;
    iDynTree::VectorDynSize sol;
    double expected;

    for (int i = 0; i <= iterations; ++i){
        t = initTime + dT*i;
        ASSERT_IS_TRUE(toBeTested.getSolution(t, sol));
        expected = x1 * std::exp(lambda1*(t - initTime));
        ASSERT_EQUAL_DOUBLE_TOL(expected, sol(0), std::abs(expected)*relTol); //up to the eight significative digit
    }

}

int main(){

    std::cerr << "Test 1" << std::endl;

    std::shared_ptr<TestEquation1> dynamicalSystem = std::make_shared<TestEquation1>(lambda1);
    dynamicalSystem->setInitialCondition(x1);

    RK4 RK4_1(dynamicalSystem);

    ForwardEuler FE_1(dynamicalSystem);

    relTol = 1E-8;
    IntegratorTest1(RK4_1);
    relTol = 5E-2;
    IntegratorTest1(FE_1);

    std::cerr << "Test 2" << std::endl;


    std::shared_ptr<TestEquation2> dynamicalSystem2 = std::make_shared<TestEquation2>(lambda1, lambda2);
    dynamicalSystem2->setInitialCondition(x1, x2);

    RK4 RK4_2(dynamicalSystem2);
    ForwardEuler FE_2(dynamicalSystem2);

    relTol = 1E-8;
    IntegratorTest2(RK4_2);
    relTol = 5E-2;
    IntegratorTest2(FE_2);


    std::cerr << "Test 3" << std::endl;

    std::shared_ptr<ControlledTestEquation> dynamicalSystemCtrl = std::make_shared<ControlledTestEquation>(lambda1);

    RK4 RK4_3(dynamicalSystemCtrl);
    ForwardEuler FE_3(dynamicalSystemCtrl);

    dynamicalSystemCtrl->setInitialCondition(x1);

    relTol = 1E-8;
    IntegratorTest3(RK4_3);
    relTol = 5E-2;
    IntegratorTest3(FE_3);


    iDynTree::VectorDynSize v1(1), v2(1), v3;
    iDynTree::getRandomVector(v1);
    iDynTree::getRandomVector(v2);
    std::vector<VectorDynSize> c1(2), c2(2), c3;
    c1[0] = v1;
    c1[1] = v2;

    std::shared_ptr<Integrator> ptr = std::make_shared<ForwardEuler>(dynamicalSystem);
    ForwardEuler direct(dynamicalSystem);

    clock_t initT, endT, sumDirect = 0, sumPtr = 0;

    ASSERT_IS_TRUE(direct.evaluateCollocationConstraint(0.0, c1, c2, dT, v3)); //for eventual memory allocation
    ASSERT_IS_TRUE(ptr->evaluateCollocationConstraint(0.0, c1, c2, dT, v3));

    initT = clock();
    for (int i = 0; i < 10000; ++i) {
        ptr->evaluateCollocationConstraint(0.0, c1, c2, dT, v3);
    }
    endT = clock();
    std::cerr << "Elapsed time (ptr): " <<  static_cast<double>(endT - initT) / CLOCKS_PER_SEC * 1000.0 <<" ms."<<std::endl;

    initT = clock();
    for (int i = 0; i < 10000; ++i) {
        direct.evaluateCollocationConstraint(0.0, c1, c2, dT, v3);
    }
    endT = clock();
    std::cerr << "Elapsed time (direct): " <<  static_cast<double>(endT - initT) / CLOCKS_PER_SEC * 1000.0 <<" ms."<<std::endl;

    return EXIT_SUCCESS;
}
