% Initialize a test suite using test functions in the body of a function
%
% This function is not intended to be used directly; instead, it should be
% used at the top of a '.m'-file where the subfunctions may contain test
%
% To include subfunctions as tests and return a test-suite based on them,
% use the following layout:
%
%     function test_suite=my_test_of_abs
%         try % assignment of 'localfunctions' is necessary in Matlab >= 2016
%             test_functions=localfunctions();
%         catch % no problem; early Matlab versions can use initTestSuite fine
%         end
%         initTestSuite;
%
%     function test_abs_scalar
%         assertTrue(abs(-1)==1)
%         assertEqual(abs(-NaN),NaN);
%         assertEqual(abs(-Inf),Inf);
%         assertEqual(abs(0),0)
%         assertElementsAlmostEqual(abs(-1e-13),0)
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

    get_example_syntax=@()sprintf('%s\n',...
                            '',...
                            '    function test_suite=test_foo',...
                            '    % tests for function ''foo''',...
                            ['        try % assignment of '...
                                    '''localfunctions'' is necessary '...
                                    'in Matlab >= 2016'],...
                            ['            test_functions='...
                                        'localfunctions();'],...
                            ['        catch % no problem; early Matlab '...
                                                'versions can use '...
                                                'initTestSuite fine'],...
                            '        end',...
                            '        initTestSuite;',...
                            '',...
                            '    function test_foo1',...
                            '        % your test code here',...
                            '',...
                            '    function test_foo2',...
                            '        % your test code here'...
                            );


    e=lasterror();
    [call_stack, idx] = dbstack('-completenames');
    if numel(call_stack)==1
        example_syntax=get_example_syntax();

        error(['The script ''%s'' must be called from within '...
                    'another function, typically using the '...
                    'following syntax:\n\n%s'...
                    ],...
                    call_stack(1).name,...
                    example_syntax);
    end

    caller_fn=call_stack(idx+1).file;
    sub_func_struct=moxunit_util_mfile_subfunctions(caller_fn);

    has_error=false;

    if moxunit_util_platform_supports('localfunctions_in_script')
        n_sub_func=numel(sub_func_struct);

        handle_candidates=cell(n_sub_func,1);
        for k=1:n_sub_func
            handle_candidates{k}=str2func(sub_func_struct(k).name);
        end

    else
        if ~exist('test_functions','var')
            func_name=call_stack(2).name;
            example_syntax=get_example_syntax();

            func_error=@()error(...
                    ['In %s:\nThe variable ''test_functions'' is '...
                    'not assigned and this version\nof Matlab does '...
                    'not support the assignment of function handles\n'...
                    'through calls by functions. \nAdapt the function '...
                    '%s so that it resembles the following '...
                    'syntax:\n%s'],...
                    func_name, func_name, example_syntax);
            handle_candidates={func_error};
            has_error=true;
        else
            handle_candidates=test_functions;
        end
    end

    test_re=moxunit_util_get_test_name_regexp();

    n_sub_func=numel(sub_func_struct);
    sub_func_names=arrayfun(@(i)sub_func_struct(i).name,1:n_sub_func,...
                            'UniformOutput',false);

    n_handles=numel(handle_candidates);
    test_suite=MOxUnitTestSuite();

    for k=1:n_handles
        handle=handle_candidates{k};

        add_test=false;

        if has_error
            name=call_stack(1).name;
            add_test=true;
        else
            name=func2str(handle);
            if moxunit_util_regexp_matches(name,test_re)
                idx=find(strcmp(name,sub_func_names));
                if ~isempty(idx)
                    assert(numel(idx)==1, 'name match should be unique');
                    add_test=sub_func_struct(idx).nargout==0;
                end
            end
        end

        if add_test
            test_case=MOxUnitFunctionHandleTestCase(...
                                                name,...
                                                caller_fn, handle);
            test_suite=addTest(test_suite, test_case);
        end

    end

    % If not called from another function, execute the test directly
    % and remove the test_suite variable
    if numel(call_stack)==2
        disp(run(test_suite));

        % Avoid showing "ans = MOxUnitTestSuite object: 1-by-1"
        % when run without explicitly assigning output to a variable
        clear test_suite;
    end

