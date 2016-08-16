function elem_str=moxunit_util_elem2str(elem, max_chars)
% Convert element to string representation
%
% expr_str=moxunit_util_elem2str(expr[, max_chars])
%
% Inputs:
%   expr            value of any type
%   max_chars       optional maximum size of the output (default: 100)
%
% Output:
%   elem_str        If the platform is Matlab and expr is 'small' (does
%                   not have a 'large' number of values), it returns the
%                   result of disp(expr). Otherwise, if expr has a size,
%                   then the size and the class of expr are returned. As a
%                   fallback, only the class is shown.
%
% Note:
%   - because evalc is not implemented in Octave (yet), it is not possible
%     to capture the output from disp in a variable;
%
% NNO, SCL 2014-2015

    if nargin<2
        max_chars=100;
    end

    try
        if is_row_vector_string(elem)
            elem_str=['''' elem ''''];
        elseif numel(elem)<=max_chars
            % try pretty printing
            elem_str=tiny_elem2str(elem);
        else
            elem_str=elem2str_size_and_class(elem);
        end
    catch
        % fall back
        elem_str=elem2str_class(elem);
    end

    elem_str=limit_string_size(elem_str,max_chars);


function elem_str=tiny_elem2str(elem)
% return a string representation of elem
% - just 's' for a row-vector string s;
% - the result of mat2str if elem is numeric or logical and elem is not
%     'large';
% - the string representation based on 'evalc' if available and
%   'elem' is not too big;
% - as a fallback, the size and class of elem.

    siz=size(elem);
    if ischar(elem) && (isequal(siz,[0 0]) || (numel(siz)==2 && siz(1)==1))
        % Strings as row vectors are trivially supported by leaving them
        % as-is. We just encapsulate in quotation marks to make it clear
        % they are a string.

        % (For compatibility with older Matlab versions, "isrow" is not
        %  used.)

        elem_str=['''' elem ''''];

    elseif isnumeric(elem)
        % If the element is numeric, we can use mat2str
        elem_str=mat2str(elem);

    elseif exist('evalc', 'builtin')
        % If we have `evalc`, we can just trust the `display` function
        % provided by the environment to format the string correctly
        % and capture it with `evalc`.
        elem_str=evalc('disp(elem)');

        % remove trailing newlines
        elem_str=regexprep(elem_str,sprintf('[\n]*$'),'');

    else
        % If evalc is not present (Octave), just show the size and
        % the class
        elem_str=elem2str_size_and_class(elem);
    end


function tf=is_row_vector_string(elem)
% return true if the input is a row vector string
%
% does not use isrow for compatibility with older versions of Matlab

    tf=false;

    if ischar(elem)
        siz=size(elem);
        tf=isequal(siz,[0 0]) || ...
                (numel(siz)==2 && siz(1)==1);
    end


function elem_str=elem2str_size_and_class(elem)
% return string representation of elem with size and class
    siz_cell = arrayfun(@num2str, size(elem), ...
                        'UniformOutput', false);
    siz_str = moxunit_util_strjoin(siz_cell, 'x');
    elem_str = sprintf('%s(%s)', siz_str, class(elem));


function elem_str=elem2str_class(elem)
% return string representation of elem with class only; used as a fallback
% option if tiny_elem2str and elem2str_size_and_class cannot be used
    elem_str = sprintf('(%s)', class(elem));


function elem_str=limit_string_size(elem_str, max_chars)
% if elem_str is longer than max_chars, replace the inner part by '...'
    if numel(elem_str)>max_chars
        n=floor(max_chars/2);
        infix=sprintf('\n...\n');
        elem_str=[elem_str(1:n) infix elem_str(end+((1-n):0))];
    end


