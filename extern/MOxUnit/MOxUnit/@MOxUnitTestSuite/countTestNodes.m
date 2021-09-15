function n=countTestNodes(obj)
% Return the number of test nodes
%
% c=countTestCases(obj)
%
% Inputs:
%   obj             MoxUnitTestSuite instance
%
% Output:
%   c               Number of test nodes in obj
%
% NNO 2015

    n=obj.test_count;