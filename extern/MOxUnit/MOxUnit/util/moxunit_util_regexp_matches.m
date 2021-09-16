function tf=moxunit_util_regexp_matches(str, pat)
% return whether a string matches a regular pattern
%
% tf=moxunit_util_regexp_matches(str, pat)
%
% Inputs:
%   str                 string to match
%   pat                 regular rexpression to match
%
% Output:
%   tf                  true if str matches the pattern pat, false
%                       otherwise
%
    check_inputs(str, pat);

    tf=~isempty(regexp(str,pat,'once'));


function check_inputs(str, pat)
    if ~ischar(str)
        error('first argument must be a string');
    end

    if ~ischar(pat)
        error('second argument must be a string');
    end
