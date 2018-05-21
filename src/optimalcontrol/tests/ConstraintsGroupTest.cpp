#include <iDynTree/Constraint.h>
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


    iDynTree::VectorDynSize upperbound(1);
    upperbound.zero();

    iDynTree::assertTrue(constraint1->setUpperBound(upperbound));
    iDynTree::assertTrue(constraint2->setUpperBound(upperbound));


    ConstraintsGroup newGroup("dummyGroup", 2);

    TimeRange range;
    iDynTree::assertTrue(range.setTimeInterval(1.0, 2.0));

    iDynTree::assertTrue(newGroup.addConstraint(constraint1, range));

    iDynTree::assertTrue(range.setTimeInterval(0.0, 1.0));

    iDynTree::assertTrue(newGroup.addConstraint(constraint2, range));

    iDynTree::VectorDynSize dummyState(1), dummyControl, expected(2), result;

    dummyState(0) = -1.0;
    expected(0) = dummyState(0);
    expected(1) = 0.0;

    iDynTree::assertTrue(newGroup.isFeasibilePoint(1.5, dummyState, dummyControl));
    iDynTree::assertTrue(newGroup.evaluateConstraints(1.5, dummyState, dummyControl, result));
    iDynTree::assertVectorAreEqual(result, expected, 1e-10,"",-1);

    dummyState(0) = -2.0;
    expected(0) = dummyState(0);
    expected(1) = 0.0;

    iDynTree::assertTrue(newGroup.isFeasibilePoint(0.5, dummyState, dummyControl));
    iDynTree::assertTrue(newGroup.evaluateConstraints(0.5, dummyState, dummyControl, result));
    iDynTree::assertVectorAreEqual(result, expected, 1e-10,"",-1);

    return true;
}


int main(){
    iDynTree::assertTrue(groupTest());
    return 0;
}
