function fs=moxunit_util_mfile_subfunctions(fn)
% find the names of subfunctions defined in an .m file
%
% names=moxunit_util_mfile_subfunctions(fn)
%
% Input:
%   fn                  filename of .m file
%
% Output:
%   fs                  Px1 struct (with P the number of subfunctions)
%                       with fields:
%                       .name      name of the function
%                       .nargout   number of output elements
%
% Example:
%    Running this function on itself returns two subfunctions,
%    'read_file' and 'apply_text_replacements'
%
% Notes:
%   - this function uses a simple pattern matching algorithm which may
%     not work correctly if some weird unexpected syntax is used
%   - nested subfunctions are returned by this funtion as well
%   - in matlab, "which('-subfun',fn)" can be used as an alternative to
%     this function; Octave does not support this usage of "which"
%
% NNO Jan 2014

    raw_text=read_file(fn);

    newline=sprintf('\n');
    dos_newline=sprintf('\r\n');

    % simplify the contents of the file
    % - convert DOS-style newlines to Unix-style newlines
    % - remove line continuations
    % - replace newlines by double newlines, so that the end of a function
    %   name can be detected through the presence of whitespace
    replacements={dos_newline, newline;...
                  ['...' newline],'';...
                  newline,[newline newline]};
    text=apply_text_replacements(replacements, raw_text);

    % function outputs can be specifed in three ways, namely with zero,
    % one, or more than one output
    argout_pat='\s*((\s([\w~]+\s*=)?)|(\[[\w~,\s]*\]\s*=))\s*';
    argout_pat='\s*(?<argout>(\s([\w~]+\s*=)?)|(\[[\w~,\s]*\]\s*=))\s*';


    % function name that we are after
    func_name_pat='(?<name>\w*)';

    % input arguments and function name
    prefix=[newline '\s*'];
    postfix='([\W$])';

    % complete pattern of a function (excluding input arguments)
    pat=sprintf('%sfunction%s%s%s',...
                        prefix,argout_pat,func_name_pat,postfix);

    % match on tokens, because using 'names' does not work with Octave in
    % combination with the nested parenthesis RE patterns as used in
    % argout_pat
    match=regexp([newline text newline],pat,'tokens');

    % extract function names
    n_func=numel(match)-1;
    names=cell(n_func,1);
    nargouts=cell(n_func,1);

    for k=1:n_func
        m=match{k+1};

        args=m{end-2};
        args=regexprep(args,'[\[,\]=]+',' ');
        args=regexprep(args,'\s+',' ');
        args=regexprep(args,'^[ ]+','');
        args=regexprep(args,'[ ]+$','');

        if isempty(args)
            arg_count=0;
        else
            arg_count=1+numel(regexp(args,' '));
        end

        names{k}=m{end-1};
        nargouts{k}=arg_count;
    end

    fs=struct('name',names,'nargout',nargouts);


function s = read_file(fn)
    fid = fopen(fn);

    if fid==-1
        error('Error reading from file %s\n', fn);
    end

    cleaner = onCleanup(@()fclose(fid));
    s = fread(fid,[1 inf],'char=>char');

function text=apply_text_replacements(replacements, text)
    for k=1:size(replacements,1)
        row=replacements(k,:);
        text = strrep(text, row{1}, row{2});
    end

