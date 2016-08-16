function string=getProgressStr(obj,verbosity)
% give description of text outcome
%
% Inputs:
%   obj             MOxUnitTestOutcome instance.
%   verbosity       integer in range 0 to 2
%
% Output:
%   string          String containing a line (if verbosity==2); or a single
%                   character ('.','F','E','s'; if verbosity==1); or the
%                   empty string

    outcome=getOutcomeStr(obj, verbosity);

    if verbosity>=2
        string=sprintf('%10s: %s\n', outcome, str(getTest(obj)));
    elseif verbosity>=1
        string=outcome;
    else
        string='';
    end

