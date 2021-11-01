function result=moxunit_runtests(varargin)
% Run MOxUnit tests suite
%
% result=moxunit_runtests(...)
%
% Inputs:
%   '-verbose'              show verbose output. If not provided,
%                           non-verbose output is shown.
%   '-quiet'                do not show output
%   filename                } test the unit tests in filename
%   directory               } (which must initialize a test suite through
%   suite                   } initTestSuite), in the directory, or the
%                           } MOxUnitTestSuite instance.
%                           Multiple filename or directory arguments can be
%                           provided. If there are no filename or directory
%                           arguments, then all tests in the current
%                           directory are run.
%
%   '-recursive'            If this option is present, then files are added
%                           recursively from any directory. If absent, then
%                           only files from each directory (but not their
%                           subdirectories) are added.
%   '-logfile', output      store the output in file output. If not
%                           provided, then output is directed to the
%                           command window
%   '-with_coverage'        record coverage using MOCov
%   '-cover', cd            record code coverage in directory cd
%   '-cover_json_file', cj  store coverage report in json file cj for
%                           processing by coveralls.io
%   '-cover_xml_file', cx   store coverage XML output in file cx
%   '-cover_html_dir, ch    store coverage HTML output in directory ch
%   '-junit_xml_file', jx   store test results in junit XML file jx
%   '-partition_index', pi  } Either both or neither of these arguments
%   '-partition_count', pc  } must be passed to this function. It causes
%                             a subset of all tests to be run, namely the
%                             ones indexed by:
%                                pi+pc*K
%                             for all values of K. Default values are
%                             pi=1 and pc=1, meaning that all tests are
%                             run. A use case is parallelization of test
%                             cases over multiple processes.
%
%
% Output:
%   result                  true if no test failed or raised an error. In
%                           other words, true if all tests were either
%                           successful or skipped. Result is true if
%                           no tests were run.
%
% Notes:
%   - This function can be run without the function syntax. For example,
%
%       moxunit_runtests
%
%     runs all tests in the current directory, and
%
%       moxunit_runtests ../tests -verbose -logfile my_log.txt
%
%     runs all tests in the tests sub-directory of the parent directory,
%     and stores verbose output in the file my_log.txt
%   - This function attempts to show similar behaviour as in
%     Steve Eddins' MATLAB xUnit Test Framework (2009-2012)
%     URL: http://www.mathworks.com/matlabcentral/fileexchange/
%                           22846-matlab-xunit-test-framework
%   - To define tests, functions can be written that use initTestSuite.
%
% See also: initTestSuite, mocov
%
% NNO Jan 2014


    params=get_params(varargin{:});

    if params.fid>2
        % not standard or error output; file must be closed
        % afterwards
        cleaner=onCleanup(@()fclose(params.fid));
    end

    suite=MOxUnitTestSuite();

    % build pattern for filenames by combining the test name pattern with
    % extension
    mfile_ext_pattern='.m$';
    mfile_test_filename_pattern=get_test_file_pattern(mfile_ext_pattern);

    suite=add_from_to_test_spec(suite, ...
                                        params.to_test_spec,...
                                        mfile_test_filename_pattern,...
                                        params.add_recursive);

    % show summary of test suite
    if params.verbosity>0
        fprintf(params.fid,'%s\n',str(suite));
    end

    % initialize test results
    suite_name=class(suite);
    test_report=MOxUnitTestReport(params.verbosity,params.fid,suite_name);

    % run all tests with helper
    test_report=run_all_tests(suite, test_report, params);

    % show summary of test result
    disp(test_report);

    if ~isempty(params.junit_xml_file)
        writeXML(test_report,params.junit_xml_file);
    end


    % return true if no errors or failures
    result=wasSuccessful(test_report);


function mfile_test_filename_pattern=get_test_file_pattern(...
                                                mfile_ext_pattern)
    test_name_pattern=moxunit_util_get_test_name_regexp();
    assert(sum(test_name_pattern=='$')==1,'unexpected pattern');
    assert(test_name_pattern(end)=='$','unexpected pattern');

    mfile_test_filename_pattern=regexprep(test_name_pattern,...
                                            '\$',...
                                            mfile_ext_pattern);


function suite=add_from_to_test_spec(suite, ...
                                        to_test_spec, ...
                                        mfile_test_filename_pattern,...
                                        add_recursive)
    for k=1:numel(to_test_spec)
        % add files to the test suite
        to_test=to_test_spec{k};
        if isa(to_test,'MOxUnitTestSuite')
            suite=addFromSuite(suite,to_test);
        elseif moxunit_util_isfolder(to_test)
            suite=addFromDirectory(suite,...
                                    to_test,...
                                    mfile_test_filename_pattern,...
                                    add_recursive);
        else
            suite=addFromFile(suite,to_test);
        end
    end



function test_report=run_all_tests(suite, test_report, params)
    f_handle=@()run(suite, test_report,...
                        params.partition_index,params.partition_count);

    with_coverage=params.with_coverage;

    if with_coverage
        pkg='mocov';
        if isempty(which(pkg))
            error(['command ''%s'' not found, '...
                    'coverage is not supported'],pkg);
        end

        mocov_expr_param={'-expression',f_handle};

        all_keys=fieldnames(params);
        msk_iscov=cellfun(@(x)~isempty(regexp(x,'^cover','once')),all_keys);
        msk_isempty=structfun(@isempty, params);
        keys=all_keys(msk_iscov & ~msk_isempty);
        n_keys=numel(keys);

        mocov_params_cell=cell(size(keys));
        for k=1:n_keys
            key=keys{k};
            value=params.(key);
            key_arg=['-' key];
            if ischar(value)
                param_elem={key_arg,value};
            elseif iscell(value)
                n_values=numel(value);
                param_elem_matrix=[repmat({key_arg},1,n_values);...
                                           value(:)'];
                param_elem=param_elem_matrix(:)';
            else
                error('moxunit:illegalParameterValue',...
                        ['Expected char or cell input.' ...
                        ' Was given a %s instead.'], ...
                        class(value));
            end

            mocov_params_cell{k}=param_elem;
        end

        mocov_params=cat(2,mocov_params_cell{:});

        all_params=[mocov_expr_param, mocov_params];

        if params.verbosity>=1
            params_as_strings=cellfun(@param2str,all_params,...
                                    'UniformOutput',false);
            params_joined=sprintf('%s ',params_as_strings{:});

            fprintf('running coverage with parameters: %s\n',...
                                params_joined);
        end


        test_report=mocov(all_params{:});
    else
        test_report=f_handle();
    end

function s=param2str(p)
    if ischar(p)
        s=p;
    else
        s=sprintf('<%s>',class(p));
    end


function params=get_params(varargin)
    % set defaults
    params.verbosity=1;
    params.fid=1;
    params.junit_xml='';
    params.add_recursive=false;
    params.cover='';
    params.cover_exclude={};
    params.cover_xml_file='';
    params.junit_xml_file='';
    params.cover_json_file='';
    params.cover_html_dir='';
    params.cover_method='';
    params.with_coverage=false;
    params.partition_index=1;
    params.partition_count=1;

    % allocate space for filenames
    n=numel(varargin);
    to_test_spec=cell(n,1);

    k=0;
    while k<n
        k=k+1;
        arg=varargin{k};

        if isa(arg,'MOxUnitTestSuite')
            to_test_spec{k}=arg;
            continue
        elseif ~ischar(arg)
            error('moxunit:illegalParameter',...
                    'Illegal argument type at position %d', k);
        end

        switch arg
            case '-verbose'
                params.verbosity=params.verbosity+1;

            case '-quiet'
                params.verbosity=params.verbosity-1;


            case '-logfile'
                if k==n
                    error('moxunit:missingParameter',...
                           'Missing parameter after option ''%s''',arg);
                end
                k=k+1;
                fn=varargin{k};

                params.fid=fopen(fn,'w');
                if params.fid==-1
                    error('Could not open file %s for writing', fn);
                end

            case {'-cover','-cover_xml_file','-junit_xml_file',...
                        '-cover_json_file','-cover_html_dir',...
                        '-cover_method'}
                params=set_key_value(params,varargin,k);
                k=k+1;

             case '-cover_exclude'
                if k==n
                    error('moxunit:missingParameter',...
                           'Missing parameter after option ''%s''',arg);
                end
                k=k+1;
                params.cover_exclude(end+1)=varargin(k);

            case '-recursive'
                params.add_recursive=true;


            case '-with_coverage'
                params.with_coverage=true;

            case '-partition_index'
                if k==n
                    error('moxunit:missingParameter',...
                           'Missing parameter after option ''%s''',arg);
                end
                k=k+1;
                value=varargin{k};
                if ischar(value)
                    value=str2double(value);
                end
                params.partition_index=value;

            case '-partition_count'
                if k==n
                    error('moxunit:missingParameter',...
                           'Missing parameter after option ''%s''',arg);
                end
                k=k+1;
                value=varargin{k};
                if ischar(value)
                    value=str2double(value);
                end
                params.partition_count=value;

            otherwise

                if ~isempty(dir(arg))
                    to_test_spec{k}=arg;
                else
                    % Close the file if it has been already open before
                    % throwing an error. This prevents resource leak and,
                    % eventually 'Too many files open' error.
                    try
                        fclose(params.fid);
                    end %#ok<TRYNC>
                    error('moxunit:illegalParameter',...
                    'Parameter not recognised or file missing: %s', arg);
                end
        end
    end

    to_test_spec=to_test_spec(~cellfun(@isempty,to_test_spec));

    if numel(to_test_spec)==0
        me_name=mfilename();
        if isequal(pwd(), fileparts(which(me_name)))
            error('moxunit:illegalParameter',...
                        ['Cannot run from the MOxUnit directory ',...
                            'because this would lead to recursive '...
                            'calls to %s. To run unit tests '...
                            'on MOxUnit itself, use:\n\n'...
                            '  %s ../tests'], me_name, me_name);
        end
        to_test_spec={pwd()};
    end

    params.to_test_spec=to_test_spec;

    check_cover_consistency(params)


function params = set_key_value(params,args,k)
    n=numel(args);

    arg = args{k};
    assert(arg(1)=='-');
    key=arg(2:end);

    if k==n
        error('moxunit:missingParameter',...
               'Missing parameter after option ''%s''',arg);
    end

    params.(key)=args{k+1};

function check_cover_consistency(params)
    keys=fieldnames(params);

    n=numel(keys);

    with_coverage=params.with_coverage;
    for k=1:n
        key=keys{k};

        if ~isempty(regexp(key,'^cover','once'))
            value=params.(key);
            if ~isempty(value) && ~with_coverage
                error('Option ''%s'' requires -with_coverage');
            end
        end
    end

    if with_coverage && isempty(params.cover)
        error('Option ''-with_coverage'' requires ''-cover''');
    end







