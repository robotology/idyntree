function obj=addTest(obj, t)
% Add MOxUnitTestNode instance to the suite
%
% obj=addTest(obj, t)
%
% Inputs:
%   obj             MoxUnitTestSuite instance
%   t               MOxUnitTestNode instance to be added to the suite.
%
% Output:
%   obj             MoxUnitTestSuite instance with the MOxUnitTestNode
%                   instance added.
%
% See also: initTestSuite
%
% NNO 2015
    n_tests=numel(obj.tests);

    obj_tests_too_small=n_tests+1 < obj.test_count;

    if obj_tests_too_small
        % double the size every time obj.tests is too small. This means
        % that adding N tests requires that obj.tests is resized
        % only ceil(log2(N)) times
        empty_idx=2*n_tests+1;
        assert(numel(obj.tests)<empty_idx);

        % allocate more space
        obj.tests{empty_idx}=[];
    end

    % store the test
    obj.test_count=obj.test_count+1;
    obj.tests{obj.test_count}=t;
