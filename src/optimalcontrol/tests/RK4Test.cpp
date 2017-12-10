#include "iDynTree/DynamicalSystem.h"
#include "iDynTree/Core/TestUtils.h"
#include "iDynTree/Core/VectorDynSize.h"
#include "iDynTree/Integrators/RK4.h"
#include "iDynTree/Controller.h"
#include <memory>
#include <cmath>
#include <iostream>

using namespace iDynTree;
using namespace iDynTree::optimalcontrol;
using namespace iDynTree::optimalcontrol::integrators;


class TestEquation1 : public DynamicalSystem{
    double m_lambda;
    VectorDynSize m_initialConditions;
public:
    TestEquation1(double lambda)
        :DynamicalSystem(1,0)
        ,m_lambda(lambda)
        ,m_initialConditions(1)
    {}

    bool dynamics(const VectorDynSize &state, double time, VectorDynSize &stateDynamics){
        if (state.size() != 1)
            return false;
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

int main(){

    double lambda1 = 1;
    double lambda2 = 0.01;
    double dT = 0.01;
    double x1 =-1.0;
    double x2 =2.0;
    double initTime = 1.0;
    double endTime = 20.0;
    double relTol = 1E-8;

    std::cerr << "Test 1" << std::endl;

    std::shared_ptr<TestEquation1> dynamicalSystem = std::make_shared<TestEquation1>(lambda1);

    RK4 integrator(dynamicalSystem);

    assertTrue(integrator.setMaximumStepSize(dT));
    dynamicalSystem->setInitialCondition(x1);
    assertTrue(integrator.integrate(initTime, endTime));

    int iterations = std::round((endTime-initTime)/dT);
    double t = initTime;
    iDynTree::VectorDynSize sol;
    double expected;

    for (int i = 0; i <= iterations; ++i){
        t = initTime + dT*i;
        assertTrue(integrator.getSolution(t, sol));
        expected = x1 * std::exp(lambda1*(t - initTime));
        assertDoubleAreEqual(expected, sol(0), std::abs(expected)*relTol); //up to the eight significative digit
    }


    std::cerr << "Test 2" << std::endl;


    std::shared_ptr<TestEquation2> dynamicalSystem2 = std::make_shared<TestEquation2>(lambda1, lambda2);

    RK4 integrator2(dynamicalSystem2);

    assertTrue(integrator2.setMaximumStepSize(dT));
    dynamicalSystem2->setInitialCondition(x1, x2);
    assertTrue(integrator2.integrate(initTime, endTime));

    for (int i = 0; i <= iterations; ++i){
        t = initTime + dT*i;
        assertTrue(integrator2.getSolution(t, sol));
        expected = x1 * std::exp(lambda1*(t - initTime));
        assertDoubleAreEqual(expected, sol(0), std::abs(expected)*relTol); //up to the eight significative digit
        expected = x2 * std::exp(lambda2*(t - initTime));
        assertDoubleAreEqual(expected, sol(1), std::abs(expected)*relTol); //up to the eight significative digit
    }

    std::cerr << "Test 3" << std::endl;

    std::shared_ptr<ControlledTestEquation> dynamicalSystemCtrl = std::make_shared<ControlledTestEquation>(lambda1);

    RK4 integrator3(dynamicalSystemCtrl);

    assertTrue(integrator3.setMaximumStepSize(dT));
    dynamicalSystemCtrl->setInitialCondition(x1);
    assertTrue(integrator3.integrate(initTime, endTime));


    for (int i = 0; i <= iterations; ++i){
        t = initTime + dT*i;
        assertTrue(integrator3.getSolution(t, sol));
        expected = x1 * std::exp(lambda1*(t - initTime));
        assertDoubleAreEqual(expected, sol(0), std::abs(expected)*relTol); //up to the eight significative digit
    }



    return EXIT_SUCCESS;
}
