function tf=isNonFailure(obj)
% return whether this test was a non-failure
%
% tf=isSuccess(obj)
%
% Input:
%   obj                     MoxUnitErroredTestOutcome object
%
% Output:
%   tf                      set to false, because an errored test is a
%                           failure
%
% See also: @MoxUnitTestOutcome

    tf=false;
