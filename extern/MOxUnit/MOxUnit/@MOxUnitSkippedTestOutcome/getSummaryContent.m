function reason=getSummaryContent(obj)
% return the reason why the test was skipped
%
% c=getSummaryContent(obj)
%
% Input:
%   obj                     MoxUnitSkippedTestOutcome object
%
% Output:
%   error_struct            String containing the reason the test was
%                           skipped
%
% See also: lasterror

    reason=obj.reason;
