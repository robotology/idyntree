function obj=addFromSuite(obj, s)
% Add MOxUnitTestSuite tests to the suite
%
% obj=addFromSuite(obj, s)
%
% Inputs:
%   obj             MoxUnitTestSuite instance
%   s               MOxUnitTestSuite instance with tests to be added to
%                   the suite.
%
% Output:
%   obj             MoxUnitTestSuite instance with all tests in s
%                   added to the suite
%
% See also: initTestSuite
%
% NNO 2015

    n_obj=countTestNodes(obj);
    n_s=countTestNodes(s);

    % allocate space
    obj.tests{end+n_s}=[];

    for k=1:n_s
        obj.tests{n_obj+k}=getTestNode(s,k);
    end
