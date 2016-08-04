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

    start_tic = tic;

    try
        passed=false;
        try
            obj.function_handle();
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
                test_outcome_args={e};
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