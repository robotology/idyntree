function error_struct=getSummaryContent(obj)
% return the error struct that made the test error
%
% c=getSummaryContent(obj)
%
% Input:
%   obj                     MoxUnitErroredTestOutcome object
%
% Output:
%   error_struct            Struct containing the lasterror() contents that
%                           made the test fail
%
% See also: lasterror

    error_struct=obj.error;
