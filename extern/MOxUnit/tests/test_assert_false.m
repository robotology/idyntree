function test_suite=test_assert_false()
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_assert_false_exceptions
    assertExceptionThrown(@()assertFalse(...
                                true),...
                                        'assertFalse:trueCondition');
    assertExceptionThrown(@()assertFalse(...
                                struct),...
                                        'assertFalse:invalidCondition');
    assertExceptionThrown(@()assertFalse(...
                                [false false]),...
                                        'assertFalse:invalidCondition');

function test_assert_false_passes
    assertFalse(false);
