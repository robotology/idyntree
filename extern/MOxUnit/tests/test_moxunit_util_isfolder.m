function test_suite=test_moxunit_util_isfolder
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;


function test_moxunit_util_isfolder_basics()
    % current direcoty
    assertTrue(moxunit_util_isfolder(pwd));
    assertTrue(moxunit_util_isfolder('.'));
    assertTrue(moxunit_util_isfolder('..'));

    % this test fails almost surely
    r = rand_str();
    assertFalse(moxunit_util_isfolder(r));

    % illegal input type
    assertExceptionThrown(@()moxunit_util_isfolder(struct));

function s=rand_str()
    n = 25;
    s=char(26*rand(1,n)+65);
