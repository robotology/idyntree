function tf=isNonFailure(obj)
% return whether this test was a non-failure
%
% tf=isSuccess(obj)
%
% Input:
%   obj                     MoxUnitPassedTestOutcome object
%
% Output:
%   tf                      set to true, because a skipped test is not a
%                           failure
%
% See also: @MoxUnitTestOutcome

    tf=true;