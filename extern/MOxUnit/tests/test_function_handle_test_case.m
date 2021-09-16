function test_suite=test_function_handle_test_case
% tests for MOxUnitFunctionHandleTestCase
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function s=randstr()
    s=char(20*rand(1,10)+65);

function test_function_handle_test_case_basics

    outcome_class2func=struct();
    outcome_class2func.MOxUnitPassedTestOutcome=@do_nothing;
    outcome_class2func.MOxUnitSkippedTestOutcome=@()...
                            moxunit_throw_test_skipped_exception('foo');
    outcome_class2func.MOxUnitFailedTestOutcome=@()error('here');

    keys=fieldnames(outcome_class2func);
    for k=1:numel(keys)
        outcome_class=keys{k};
        func=outcome_class2func.(outcome_class);

        name=randstr();
        location=randstr();

        f=MOxUnitFunctionHandleTestCase(name, location, func);
        assertEqual(getName(f),name);
        assertEqual(getLocation(f),location);
        assertEqual(str(f),sprintf('%s:  %s',name,location));

        rep=MOxUnitTestReport(0,1);
        rep=run(f,rep);

        assertEqual(countTestOutcomes(rep),1);
        outcome=getTestOutcome(rep,1);

        assertEqual(class(outcome),outcome_class);
    end

function test_function_handle_test_case_reset_warning()
    if moxunit_util_platform_is_octave()
        reason=['resetting the warning state seems not to work ' ...
                '(TODO: file a bug report?)'];
        moxunit_throw_test_skipped_exception(reason);
        return;
    end

    s=warning('query');
    state_resetter=onCleanup(@()warning(s));

    % generate unique id
    id=sprintf('%s:%s:%s',randstr(),randstr(),randstr());

    assertEqual(get_warning_state(id),[])

    name=randstr();
    location=randstr();
    func=@()warning('off',id);
    f=MOxUnitFunctionHandleTestCase(name, location, func);
    rep=MOxUnitTestReport(0,1);
    run(f,rep);

    assertEqual(get_warning_state(id),[])

function s=get_warning_state(id)
% return empty array if warning state not present, or 'on' or 'off'
    w=warning('query');
    idx=find(strcmp(id,{w.identifier}))';

    if isempty(idx)
        s=[];
        return;
    end

    assert(numel(idx)==1);
    s=w(idx).state;


function disable_warning(id)
    warning('off',id);

function do_nothing()
    % do nothing
