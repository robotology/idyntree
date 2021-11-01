function report=run(obj,report)
% Run test associated with MoxUnitFunctionHandleTestCase
%
% report=run(obj,report)
%
% Inputs:
%   obj             MoxUnitFunctionHandleTestCase object
%   report          MoxUnitTestReport instance to which test results are to
%                   be reported.
%
% Output:
%   report          MoxUnitTestReport containing tests results
%                   after running the test associated with obj.
%
% See also: MoxUnitTestReport
%
% NNO 2015

    original_warning_state=warning('query');
    warning_state_resetter=onCleanup(@()warning(original_warning_state));

    start_tic = tic;

    try
        passed=false;
        try
            if nargin(obj.function_handle) > 0
                obj.function_handle(obj);
            else
                obj.function_handle();
            end
            passed=true;
        catch
            e=lasterror();

            if moxunit_isa_test_skipped_exception(e)

                last_newline_pos=find(e.message==sprintf('\n'),1,'last');
                if isempty(last_newline_pos)
                    last_newline_pos=0;
                end

                reason=e.message((last_newline_pos+1):end);

                test_outcome_constructor=@MOxUnitSkippedTestOutcome;
                test_outcome_args={reason};
            else
                test_outcome_constructor=@MOxUnitFailedTestOutcome;

                % trim the stack so that all test case machinery is
                % removed from the stack
                e_trimmed=trim_stack(e);
                test_outcome_args={e_trimmed};
            end
        end

        if passed
            test_outcome_constructor=@MOxUnitPassedTestOutcome;
            test_outcome_args={};
        end
    catch
        e=lasterror();
        test_outcome_constructor=@MOxUnitErroredTestOutcome;
        test_outcome_args={e};
    end

    test_outcome = test_outcome_constructor(obj, toc(start_tic), ...
                            test_outcome_args{:});

    report = reportTestOutcome(report, test_outcome);


function e_trimmed=trim_stack(e)
% trim the stack from e.stack, so that everything up to and including the
% first (nearest to the calling root) entry is removed
    stack=e.stack;

    n_stack=numel(stack);

    this_file=sprintf('%s.m',mfilename('fullpath'));

    for pos=n_stack:-1:1
        if strcmp(stack(pos).file,this_file)
            % found first match with this filename, now trim this function
            % and their callig functions
            trimmed_stack=stack(1:(pos-1));

            e_trimmed=e;
            e_trimmed.stack=trimmed_stack;
            return;
        end
    end

    assert(false,'This should not happen');

