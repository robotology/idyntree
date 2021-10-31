function obj=addFromFile(obj,fn)
% Add unit tests from file
%
% obj=addFromFile(obj,fn)
%
% Inputs:
%   obj             MoxUnitTestSuite instance.
%   fn              name of file that contains a top-level function that
%                   returns a MOxUnitTestNode. In a typical use case, this
%                   is a function that returns a test_suite variable using
%                   initTestSuite.
%
% Output:
%   obj             MoxUnitTestSuite instance with the MOxUnitTestNode test
%                   added, if present.
%
% See also: initTestSuite
%
% NNO 2015

    orig_pwd=pwd();
    cleaner=onCleanup(@()cd(orig_pwd));

    [fn_dir,name]=fileparts(fn);

    if ~isempty(fn_dir)
        cd(fn_dir);
    end

    func=str2func(name);

    if nargout(func)~=1
        return;
    end

    test_case=func();
    if isa(test_case,'MOxUnitTestNode')
        obj=addTest(obj,test_case);

    elseif isa(test_case,'matlab.unittest.Test')

        matlab_test_struct=test_case;
        n_tests=numel(matlab_test_struct);

        for k=1:n_tests
            test_node=MOxUnitMatlabUnitWrapperTestCase(...
                                    matlab_test_struct(k));
            obj=addTest(obj,test_node);
        end
    else
        % unknown class, silent skip
    end