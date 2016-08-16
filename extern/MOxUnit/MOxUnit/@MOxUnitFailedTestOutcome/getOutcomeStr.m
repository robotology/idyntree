function outcome=getOutcomeStr(obj, verbosity)
% give string indicating the outcome of the text
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
%                       1: 'F'
%                       2: 'failure'
%

    outcome_cell={'','F','failure'};
    outcome=outcome_cell{verbosity+1};
