function c=countTestCases(obj)
% Return the number of test cases
%
% c=countTestCases(obj)
%
% Inputs:
%   obj             MoxUnitTestSuite instance
%
% Output:
%   c               Number of test cases in obj
%
% NNO 2015

    c=0;
    for k=1:numel(obj.tests)
        t=obj.tests{k};
        c=c+countTestCases(t);
    end

