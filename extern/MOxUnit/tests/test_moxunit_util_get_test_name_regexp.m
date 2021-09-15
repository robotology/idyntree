function test_suite=test_moxunit_util_get_test_name_regexp()
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_moxunit_util_get_test_name_basics()
    matches={'test_foo','TestFoo','FooTest','foo_test'};
    non_matches={'te_st','foo','bar','','foo_test_bar','fTestb'};

    pat=moxunit_util_get_test_name_regexp();
    assert_matches_is(matches,pat,true);
    assert_matches_is(non_matches,pat,false);


function test_moxunit_util_get_test_name_ends_with_dollar_sign()
    pat=moxunit_util_get_test_name_regexp();
    assertEqual(sum(pat=='$'),1);
    assertTrue(pat(end)=='$');


function assert_matches_is(inputs,pat,expected)
    for k=1:numel(inputs)
        m=regexp(inputs{k},pat,'once');

        assertEqual(~isempty(m),expected,...
                        sprintf('fail for %s',inputs{k}));
    end
