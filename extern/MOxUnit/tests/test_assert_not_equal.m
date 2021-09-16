function test_suite=test_assert_not_equal()
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_assert_not_equal_exceptions
    assertExceptionThrown(@()assertNotEqual(...
                                [1],[1]),...
                                        'assertNotEqual:equal');
    assertExceptionThrown(@()assertNotEqual(...
                                struct(),struct()),...
                                        'assertNotEqual:equal');

function test_assert_not_equal_passes
    assertNotEqual(1,2);
    assertNotEqual([1 2],[1;2]);
    assertNotEqual(sparse(1),1);
    assertNotEqual([1 2],[1 3]);
    assertNotEqual('a','b');
