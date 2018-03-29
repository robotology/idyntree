function test_suite=JointUnitTest
    initTestSuite

function test_joint_constructor
    wrench   = iDynTree.Wrench();
    revJoint = iDynTree.RevoluteJoint();


