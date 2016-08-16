function s=str(obj)
% Return string representation of test suite.
%
% s=str(obj)
%
% Inputs:
%   obj             MoxUnitTestSuite instance
%
% Output:
%   s               String representation showing the number of tests in the
%                   test suite.
%
% NNO 2015

    s=sprintf('suite: %d tests', countTestCases(obj));
