function test_suite=test_testcase_class()
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_assert_forwarding(t)
    t.assertElementsAlmostEqual(1, 1+eps);
    t.assertEqual(1, 1);
    t.assertError(...
        @()assertEqual([1],'a'), 'assertEqual:classNotEqual');
    t.assertExceptionThrown(...
        @()assertEqual([1],'a'), 'assertEqual:classNotEqual');
    t.assertFalse(true == false);
    t.assertLessThan(0, 1);
    t.assertGreaterThan(1, 0);
    t.assertTrue(false == false);
    t.assertVectorsAlmostEqual(ones(2,1), ones(2,1)+eps);
    t.assertWarning(@()warning('moxunit:warning','msg'),...
                    'moxunit:warning');

function test_undefined_forward(t)
    assertExceptionThrown(@()t.assertDoesNotExist(1, 2),...
                          'moxunit:undefinedMethod');