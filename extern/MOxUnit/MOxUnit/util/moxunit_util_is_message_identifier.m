function tf = moxunit_util_is_message_identifier(id)
% returns whether the input string is a message identifier
%
% tf = moxunit_util_is_message_identifier(string)
%
% Input:
%   id              input string
%
% Output
%   tf              true if the input string is a message identifier. A
%                   message identifier is a string with the following
%                   properties:
%                   - contains only alphanumeric characters, and/or the
%                     underscore character ('_'), colon (':'), or
%                     hyphen ('-') [*].
%                   - it contains at least one colon
%                   - the first character is an alphabetic character
%                   - every colon is immediately followed by an alphabetic
%                     character
%
% Notes:
%   * The hyphen is allowed in Octave message identifiers, and actually
%     used in various Octave exceptions (such as 'Octave:undef-func'). In
%     Matlab the hyphen is not allowed, however there seem to be few
%     realistic, real world use cases where allowing it would change
%     behaviour of functions. We would like to be consistent across the
%     Matlab and Octave platforms, and with hyphens used often in Octave,
%     this function also consideres hyphens to be part of a message
%     identifier

    if ~ischar(id)
        error('illegal input: first argument must be char');
    end

    alpha_pat = '[a-zA-Z]';
    word_pat = '[a-zA-Z0-9_-]';

    id_pat = sprintf('(%s(%s*))',alpha_pat,word_pat);
    pat = sprintf('^%s(:%s)+$',id_pat,id_pat);

    tf = ~isempty(regexp(id, pat, 'once'));
