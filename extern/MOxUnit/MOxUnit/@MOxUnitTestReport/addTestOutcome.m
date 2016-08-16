function obj=addTestOutcome(obj, test_outcome)
% adds the test outcome to the stream of the current object
%
% obj=reportTestOutcome(obj, test_outcome)
%
% Inputs:
%   obj             MOxUnitTestReport instance
%   test_outcome    MOxUnitTestReport instance with the outcome to add.
%
% Output:
%   obj             MOxUnitTestReport instance with the test_outcome added

    obj.test_outcomes{end+1}=test_outcome;