function test_suite=test_test_node
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_test_node_basics
    rand_str=@()char(20*rand(1,10)+65);

    name=rand_str();
    nd=MOxUnitTestNode(name);

    assertEqual(getName(nd),name);
    assertFalse(isempty(strfind(str(nd), 'MOxUnitTestNode')));

