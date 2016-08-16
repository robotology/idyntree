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

    obj.tests{end+1}=t;
