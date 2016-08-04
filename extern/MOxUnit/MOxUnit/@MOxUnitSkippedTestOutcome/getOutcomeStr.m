function outcome=getOutcomeStr(obj, verbosity)
% give string indicating the outcome of the test
%
% outcome=getOutcomeStr(obj, verbosity)
%
% Inputs:
%   obj                 MOxUnitTestOutcome object
%   verbosity           Integer in the range [0,1,2]
%
% Returns:
%   outcome:            Depending on the value of verbosity
%                       0: ''
%                       1: 's'
%                       2: 'skipped'
%

    outcome_cell={'','s','skipped'};
    outcome=outcome_cell{verbosity+1};
