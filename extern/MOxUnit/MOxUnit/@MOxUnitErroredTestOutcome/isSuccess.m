function tf=isSuccess(obj)
% return whether this test was a succes
%
% tf=isSuccess(obj)
%
% Input:
%   obj                     MoxUnitErroredTestOutcome object
%
% Output:
%   tf                      set to false, because an errored test is not a
%                           success
%
% See also: @MoxUnitTestOutcome

    tf=false;