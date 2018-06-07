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
#include <iDynTree/Constraint.h>
#include <iDynTree/LinearConstraint.h>
#include <iDynTree/ConstraintsGroup.h>
#include <iDynTree/TimeRange.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/TestUtils.h>
#include <memory>

using namespace iDynTree;
using namespace iDynTree::optimalcontrol;

class DummyConstraint1 : public Constraint{
public:
    DummyConstraint1(const std::string& name) : Constraint(1,name){}

    bool evaluateConstraint(double time, const VectorDynSize &state, const VectorDynSize &control, VectorDynSize &constraint){
        if (state.size() != 1){
            return false;
        }
        constraint.resize(1);
        constraint(0) = state(0);
        return true;
    }
};

bool groupTest(){
    std::shared_ptr<DummyConstraint1> constraint1 = std::make_shared<DummyConstraint1>("Dummy1");
    std::shared_ptr<DummyConstraint1> constraint2 = std::make_shared<DummyConstraint1>("Dummy2");
    std::shared_ptr<iDynTree::optimalcontrol::LinearConstraint> constraint3;
    constraint3 = std::make_shared<iDynTree::optimalcontrol::LinearConstraint>(1, "LinConstraint");
    iDynTree::MatrixDynSize constraintMatrix(1,1);
    constraintMatrix(0, 0) = 1;
    ASSERT_IS_TRUE(constraint3->setStateConstraintMatrix(constraintMatrix));

    iDynTree::VectorDynSize upperbound(1);
    upperbound.zero();

    ASSERT_IS_TRUE(constraint1->setUpperBound(upperbound));
    ASSERT_IS_TRUE(constraint2->setUpperBound(upperbound));
    ASSERT_IS_TRUE(constraint3->setUpperBound(upperbound));


    ConstraintsGroup newGroup("dummyGroup", 2);
    ConstraintsGroup linearGroup("linGroup", 2);

    TimeRange range;
    ASSERT_IS_TRUE(range.setTimeInterval(1.0, 2.0));

    ASSERT_IS_TRUE(newGroup.addConstraint(constraint1, range));

    ASSERT_IS_TRUE(linearGroup.addConstraint(constraint3, range));

    ASSERT_IS_TRUE(linearGroup.isLinearGroup());

    ASSERT_IS_TRUE(range.setTimeInterval(0.0, 1.0));

    ASSERT_IS_TRUE(newGroup.addConstraint(constraint2, range));

    iDynTree::VectorDynSize dummyState(1), dummyControl, expected(2), result;

    dummyState(0) = -1.0;
    expected(0) = dummyState(0);
    expected(1) = 0.0;

    ASSERT_IS_TRUE(newGroup.isFeasibilePoint(1.5, dummyState, dummyControl));
    ASSERT_IS_TRUE(newGroup.evaluateConstraints(1.5, dummyState, dummyControl, result));
    ASSERT_EQUAL_VECTOR_TOL(result, expected, 1e-10);

    ASSERT_IS_TRUE(linearGroup.isFeasibilePoint(1.5, dummyState, dummyControl));
    ASSERT_IS_TRUE(linearGroup.evaluateConstraints(1.5, dummyState, dummyControl, result));
    ASSERT_EQUAL_VECTOR_TOL(result, expected, 1e-10);

    dummyState(0) = -2.0;
    expected(0) = dummyState(0);
    expected(1) = 0.0;

    ASSERT_IS_TRUE(newGroup.isFeasibilePoint(0.5, dummyState, dummyControl));
    ASSERT_IS_TRUE(newGroup.evaluateConstraints(0.5, dummyState, dummyControl, result));
    ASSERT_EQUAL_VECTOR_TOL(result, expected, 1e-10);

    return true;
}


int main(){
    ASSERT_IS_TRUE(groupTest());
    return 0;
}
