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

    n_test_nodes=countTestNodes(s);

    for k=1:n_test_nodes
        test_node=getTestNode(s, k);
        obj=addTest(obj, test_node);
    end
