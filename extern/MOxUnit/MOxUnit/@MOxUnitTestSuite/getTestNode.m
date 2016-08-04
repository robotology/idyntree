function test_node=getTestNode(obj,i)
% return a single MoxUnitTestNode
%
% test_node=getTestNode(obj,i)
%
% Inputs:
%   obj             MoxUnitTestSuite instance
%   i               index, which must be between 1 and countTestNodes(obj)
%                   (inclusive)
%
% Output:
%   test_node       The i-th MoxUnitTestNode present in obj
%
% NNO 2015

    test_node=obj.tests{i};