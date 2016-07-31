function tf=isSuccess(obj)
% return whether this test was a succes
%
% tf=isSuccess(obj)
%
% Input:
%   obj                     MoxUnitFailedTestOutcome object
%
% Output:
%   tf                      set to false, because a failed test is not a
%                           success
%
% See also: @MoxUnitTestOutcome

    tf=false;