function test_suite=JointUnitTest
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite

function test_joint_constructor
    wrench   = iDynTree.Wrench();
    revJoint = iDynTree.RevoluteJoint();


