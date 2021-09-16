function test_suite=test_moxunit_isa_test_skipped_exception
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_moxunit_isa_test_skipped_exception_basics()
    try
        moxunit_throw_test_skipped_exception('foo');
        assert(false,'should not come here');
    catch
        e=lasterror();
    end

    assert_matches(true,e.identifier);
    assert_matches(false,e.identifier(2:end));
    assert_matches(false,'skipped');

function assert_matches(tf,x)
    e=struct();
    e.identifier=x;

    assertEqual(tf,moxunit_isa_test_skipped_exception(e))
