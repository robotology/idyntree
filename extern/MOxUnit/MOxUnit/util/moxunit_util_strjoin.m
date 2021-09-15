function joined=moxunit_util_strjoin(strings, delim)
% joins strings using a delimeter string
%
% joined=moxunit_util_strjoin(strings[, delim])
%
% Inputs:
%   strings   1xP cell with strings to be joined. Each string should be a
%             row vector of characters
%   delim     delimeter string, or a cell of strings. In the
%             latter case delim should have P-1 elements. If omitted a
%             single space character ' ' is used.
%             delim can contain backslash-escaped characters that are
%             interpreted by sprintf; for example '\t' represents a tab
%             character, '\n' a newline character, and '\\' a backslash
%             character
%
% Output
%   joined    the string formed by alternating between strings and delim;
%             if delim is a string, joined=[strings{1} delim strings{2} ...
%                                            strings{P}].
%             if delim is a cell, joined=strings{1} delim{1} strings{2} ...
%                                            strings{P}
%
% Examples:
%   moxunit_util_strjoin({'a','b','c'})
%   %|| 'a b c'
%
%   moxunit_util_strjoin({'a','b','c'}, '>#<')
%   %|| 'a>#<b>#<c'
%
%   % '\\' is the escaped backslash character
%   moxunit_util_strjoin({'a','b','c'}, '\\')
%   %|| 'a\b\c'
%
%   moxunit_util_strjoin({'a','b','c'},{'*','='})
%   %|| 'a*b=c'
%
% Notes:
%   - this function implements similar functionality as matlab's strjoin
%     function introduced in 2013. this function is provided for
%     compatibility with older versions of matlab.
%   - this function was taken from cosmo_strjoin in CoSMoMVPA
%     (cosmomvpa.org), written Sep 2013

    if nargin<2
        delim=' ';
    end

    if ~iscellstr(strings)
        error('first input must be cell of strings');
    end

    n=numel(strings);

    if n==0
        joined='';
        return;
    end

    if ischar(delim)
        delim=repmat({delim},1,n-1);
    else
        if ~iscellstr(delim)
            error('second input must be string, or cell of strings');
        else
            ndelim=numel(delim);
            if ndelim+1 ~= n
                error('number of delimiters should be %d, found %d',...
                         n, ndelim);
            end
        end
    end

    % make space for output
    joined_cells=cell(1,n*2-1);

    % set string and delim values alternatingly
    for k=1:n
        joined_cells{k*2-1}=strings{k};
        if k<n
            joined_cells{k*2}=sprintf(delim{k});
        end
    end

    % join them
    joined=[joined_cells{:}];

