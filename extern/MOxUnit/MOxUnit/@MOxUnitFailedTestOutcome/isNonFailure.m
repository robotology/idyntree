function tf=isNonFailure(obj)
% return whether this test was a non-failure
%
% tf=isSuccess(obj)
%
% Input:
%   obj                     MoxUnitFailedTestOutcome object
%
% Output:
%   tf                      set to false, because a failed test is a
%                           failure
%
% See also: @MoxUnitTestOutcome

    tf=false;
