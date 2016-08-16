function error_struct=getSummaryContent(obj)
% return the error struct that made the test fail
%
% c=getSummaryContent(obj)
%
% Input:
%   obj                     MoxUnitFailedTestOutcome object
%
% Output:
%   error_struct            Struct containing the lasterror() contents that
%                           made the test fail
%
% See also: lasterror

    error_struct=obj.error;
