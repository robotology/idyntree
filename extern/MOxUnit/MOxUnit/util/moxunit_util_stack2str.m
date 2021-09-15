function s=moxunit_util_stack2str(stack,prefix)
% Return string representation of calling stack
%
% s=moxunit_util_stack2str(stack)
%
% Inputs:
%   stack                   Nx1 struct with fields .name, .line and .file
%   prefix                  (optional) prefix for each line; if omitted the
%                           empty string ('') is used.
%
% Output:
%   s                       String representation of the stack, showing one
%                           element per line. The represention is without a
%                           trailing newline
%
    if nargin<2
        prefix='';
    end

    check_input(stack);
    n=numel(stack);

    line_pat='%s:%d (%s)';
    elem2str=@(x) [prefix, sprintf(line_pat,x.name,x.line,x.file)];

    lines=arrayfun(@(i) elem2str(stack(i)),1:n,...
                    'UniformOutput',false);
    s=moxunit_util_strjoin(lines,sprintf('\n'));


function check_input(stack)
    if ~isstruct(stack)
        error('Input must be a struct');
    end

    required_keys={'file','name','line'};
    missing_keys=setdiff(required_keys,fieldnames(stack));

    if ~isempty(missing_keys)
        error('Missing key in stack: ''%s''',missing_keys{1});
    end

