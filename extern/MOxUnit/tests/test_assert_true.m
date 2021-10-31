function test_suite=test_assert_true()
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_assert_true_exceptions

    assertExceptionThrown(@()assertTrue(...
                                false),...
                                        'assertTrue:falseCondition');
    assertExceptionThrown(@()assertTrue(...
                                struct),...
                                        'assertTrue:invalidCondition');
    assertExceptionThrown(@()assertTrue(...
                                [true true]),...
                                        'assertTrue:invalidCondition');

function test_assert_true_passes
    assertTrue(true);

