function test_suite=test_test_outcome
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_test_outcome_abstract_class
    test_=rand(3);
    duration=rand(1);
    outcome=MOxUnitTestOutcome(test_,duration);

    assertEqual(test_,getTest(outcome));
    assertEqual(duration,getDuration(outcome));

    if moxunit_util_platform_is_octave
        undef_err_id='Octave:undefined-function';
    else
        undef_err_id='MATLAB:UndefinedFunction';
    end

    assertExceptionThrown(@()getSummaryStr(outcome,'text'),undef_err_id);
    assertExceptionThrown(@()getSummaryStr(outcome,'xml'),undef_err_id);
    assertExceptionThrown(@()getProgressStr(outcome,'text'),undef_err_id);

function test_test_passed_outcome
    helper_test_outcome(@MOxUnitPassedTestOutcome,{},...
                            [],'',...
                            true,true,...
                            {'','.','passed'});

function test_test_skipped_outcome
    reason=rand_str();

    helper_test_outcome(@MOxUnitSkippedTestOutcome,{reason},...
                            reason,['skipped: ' reason],...
                            false,true,...
                            {'','s','skipped'});

function test_test_error_outcome
    [error_,msg]=rand_error_struct_and_msg();

    helper_test_outcome(@MOxUnitErroredTestOutcome,{error_},...
                            error_,['error: ' msg],...
                            false,false,...
                            {'','E','error'});

function test_test_failure_outcome
    [error_,msg]=rand_error_struct_and_msg();

    helper_test_outcome(@MOxUnitFailedTestOutcome,{error_},...
                            error_,['failure: ' msg],...
                            false,false,...
                            {'','F','failure'});

function [error_,msg_text, msg_xml_cell]=rand_error_struct_and_msg()
    error_=struct();
    error_.message=rand_str();
    error_.identifier=rand_str();

    stack=struct();
    stack.name=rand_str();
    stack.line=round(rand()*10);
    stack.file=rand_str();
    error_.stack=stack;

    msg_text=sprintf('%s\n  %s:%d (%s)',...
                        error_.message,...
                        stack.name, stack.line, stack.file);


function helper_test_outcome(class_,args,...
                                content,summary_text,...
                                is_success,is_non_failure,...
                                outcome_cell)

    name=rand_str();
    test_=MOxUnitFunctionHandleTestCase(name,rand_str,@abs);
    duration=rand(1);
    c=class_(test_,duration,args{:});
    assertEqual(getSummaryContent(c),content);
    assertEqual(getSummaryStr(c,'text'),summary_text);

    str_xml=getSummaryStr(c,'xml');
    xml_sub_str=sprintf('name="%s"',name);
    assert_contains(str_xml, xml_sub_str);
    if ~is_non_failure
        % must have failed
        xml_sub_str=sprintf('<%s message=',outcome_cell{3});
        assert_contains(str_xml,xml_sub_str);
    elseif ~is_success
        xml_sub_str=sprintf('<%s',outcome_cell{3});
        assert_contains(str_xml,xml_sub_str);
    end

    assertEqual(isSuccess(c),is_success);
    assertEqual(isNonFailure(c),is_non_failure);

    for verbosity=1:3
        assertEqual(getOutcomeStr(c,verbosity-1),outcome_cell{verbosity});
    end

function s=rand_str()
    s=char(20*rand(1,10)+65);

function assert_contains(a,b)
    assert(~isempty(strfind(a,b)))

