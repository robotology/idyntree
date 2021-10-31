function test_suite=test_matlab_unittest_test_wrapper_suite
% tests for MOxUnitFunctionHandleTestCase
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_matlab_unittest_test_test_suite_basics
    classname='matlab.unittest.Test';
    if isempty(which(classname))
        reason=sprintf('<%s> class is not available',classname);
        moxunit_throw_test_skipped_exception(reason);
        return;
    end

    if ~moxunit_util_platform_supports('diagnostics_recording_plugin')
        reason=sprintf('DiagnosticsRecordingPlugin is not available');
        moxunit_throw_test_skipped_exception(reason);
        return;
    end

    [fn,is_passing]=helper_build_matlab_unittest_test();
    cleaner=onCleanup(@()delete_path_and_file_and_dir(fn));

    [pth,nm]=fileparts(fn);
    addpath(pth);

    func=str2func(nm);
    matlab_test_struct=func();

    ntests=numel(is_passing);
    assertEqual(ntests,numel(matlab_test_struct));

    % test individual tests
    for k=1:ntests
        mox_test=MOxUnitMatlabUnitWrapperTestCase(matlab_test_struct(k));
        assertTrue(isa(mox_test,'MOxUnitTestCase'));
        assertEqual(getName(mox_test),matlab_test_struct(k).Name);

        report=MOxUnitTestReport(0,1);
        report=run(mox_test,report);
        assertEqual(countTestOutcomes(report),1);
        s=wasSuccessful(report);
        assertEqual(s,is_passing(k));
    end

    % error if more than a single test case in wrapper
    assertExceptionThrown(@()...
                MOxUnitMatlabUnitWrapperTestCase(matlab_test_struct),'');

    % add entire suite
    suite=MOxUnitTestSuite();
    suite=addFromFile(suite,fn);

    assertEqual(countTestNodes(suite),ntests);

    report=MOxUnitTestReport(0,1);
    report=run(suite,report);

    assertEqual(countTestOutcomes(report),ntests);

    for k=1:ntests
        node=getTestNode(suite,k);
        assert(isa(node,'MOxUnitMatlabUnitWrapperTestCase'));

        outcome=getTestOutcome(report,k);
        s=isSuccess(outcome);
        assertEqual(s,is_passing(k));
    end




function test_matlab_unittest_test_wrapper_exceptions()
    aet=@(varargin)assertExceptionThrown(@()...
                    MOxUnitMatlabUnitWrapperTestCase(varargin{:}),'');
    aet('foo');
    aet(struct);


function delete_path_and_file_and_dir(fn)
    pth=fileparts(fn);
    rmpath(pth);

    delete(fn);
    rmdir(pth);

function [fn,is_passing]=helper_build_matlab_unittest_test()
    ntests=ceil(rand()*10+10);

    is_passing=false(ntests,1);
    lines=cell(ntests,1);

    for k=1:ntests
        is_ok=rand()>.5;

        is_passing(k)=is_ok;

        header=sprintf('function test_%d(unused)',k);
        if is_ok
            line=sprintf('abs(2);');
        else
            line='error(''foo'')';
        end

        lines{k}=sprintf('%s\n%s\n\n',header,line);
    end

    nm='test_example';
    test_header=sprintf(['function tests=%s\n'...
                    'tests=functiontests(localfunctions);\n'],nm);

    subdir=tempname();
    mkdir(subdir);

    fn=fullfile(subdir,[nm '.m']);
    fid=fopen(fn,'w');
    closer=onCleanup(@()fclose(fid));

    fprintf(fid,test_header);
    fprintf(fid,'%s',lines{:});

