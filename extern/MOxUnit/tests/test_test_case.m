function test_suite=test_test_case
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_test_case_basics
    rand_str=@()char(20*rand(1,10)+65);

    name=rand_str();
    location=rand_str();
    case_=MOxUnitTestCase(name,location);

    assertEqual(location,getLocation(case_));
    assertEqual(name,getName(case_));

    assert(~isempty(strfind(str(case_),'MOxUnitTestCase')));

function test_test_case_run
    case_=MOxUnitTestCase('foo','bar');
    assertExceptionThrown(@()run(case_),'moxunit:notImplemented');