% Initialize a test suite using test functions in the body of a function
%
% This function is not intended to be used directly; instead, it should be
% used at the top of a '.m'-file where the subfunctions may contain test
%
% To include subfunctions as tests and return a test-suite based on them,
% use the following layout:
%
%     function test_suite=my_test_of_abs
%         initTestSuite
%
%     function test_abs_scalar
%         assertTrue(abs(-1)==1)
%         assertEqual(abs(-NaN),NaN);
%         assertEqual(abs(-Inf),Inf);
%         assertEqual(abs(0),0)
%          assertElementsAlmostEqual(abs(-1e-13),0)
%
%     function test_abs_vector
%         assertEqual(abs([-1 1 -3]),[1 1 3]);
%
%     function test_abs_exceptions
%         if moxunit_util_platform_is_octave()
%             assertExceptionThrown(@()abs(struct),'');
%         else
%             assertExceptionThrown(@()abs(struct),...
%                                    'MATLAB:UndefinedFunction');
%         end
%
% After suite=my_test_of_abs(), suite will be a test suite testing
% the test_abs_scalar and test_abs_vector subfunctions
%
% Each test function should either start with 'test_' or end with '_test',
% and have no output arguments
%
% It is absolutely necessary that the main function has one output, and
% that the output variable is named 'test_suite'
%
% NNO Jan 2014


    [call_stack, idx] = dbstack('-completenames');

    caller_fn=call_stack(idx+1).file;

    subfunction_names=moxunit_util_mfile_subfunctions(caller_fn);

    n=numel(subfunction_names);
    tests=cell(n,1);

    test_suite=MOxUnitTestSuite();

    for k=1:n
        subfunction_name=subfunction_names{k};
        if moxunit_util_is_test_function_name(subfunction_name)
            try
                func=str2func(subfunction_name);
                if nargout(func)==0
                    test_case=MOxUnitFunctionHandleTestCase(...
                                                    subfunction_name,...
                                                    caller_fn, func);
                    test_suite=addTest(test_suite, test_case);
                end
            catch
            end
        end
    end

    % If the user didn't request an output, immediately execute the test and
    % display its result
    if ~nargout
        disp(run(test_suite));

        % Avoid showing "ans = MOxUnitTestSuite object: 1-by-1"
        % when run without explicitly assigning output to a variable
        clear test_suite;
    end