function c=countTestOutcomes(obj)
% return the number of TestOutcome objects in this instance
%
% c=countTestOutcomes(obj)
%
% Inputs:
%   obj             MOxUnitTestReport instance
%
% Output:
%   c               Integer representing the number of TestOutcome objects
%                   in this instance. This number is equal to the number of
%                   times that addTestOutcome and/or reportTestOutcome was
%                   called on the obj instance.

    c=numel(obj.test_outcomes);