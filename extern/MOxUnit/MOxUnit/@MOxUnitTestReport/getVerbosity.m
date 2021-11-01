function verbosity=getVerbosity(obj)
% get the verbosity of the report
%
% verbosity=getVerbosity(obj)
%
% Input:
%   obj                 MOxUnitTestReport instance
%
% Output:
%   verbosity           Integer indicating how verbose the report is;
%                       higher values means more verbose

    verbosity=obj.verbosity;
