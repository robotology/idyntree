function obj=addFromDirectory(obj,directory,pat,add_recursive)
% Add unit tests from directory
%
% obj=addFromFile(obj,directory[,pat])
%
% Inputs:
%   obj             MoxUnitTestSuite instance.
%   directory       name of directory that may contain files with top-level
%                   function that returns MOxUnitTestNode instances.
%   pat             File pattern to look for in directory (default: '*.m').
%                   Matching files are added using addFromFile.
%   add_recursive   If true, then files are added recursively from
%                   sub-directories. If false (the default) then only files
%                   are added from directory, but files in subdirectories
%                   are ignored.
%
% Output:
%   obj             MoxUnitTestSuite instance with MOxUnitTestNode
%                   instances  added, if present in the files in directory.
%
% Notes:
%   - this function does not add files recursively.
%
% See also: initTestSuite, addFromFile
%
% NNO 2015

    if nargin<4 || isempty(add_recursive)
        add_recursive=false;
    end

    if nargin<3 || isempty(pat)
        pat='*.m';
    end

    pat_regexp=['^' regexptranslate('wildcard',pat) '$'];

    if isdir(directory)
        d=dir(directory);
        n=numel(d);

        for k=1:n
            fn=d(k).name;
            path_fn=fullfile(directory,fn);

            if isdir(path_fn)
                if add_recursive && is_sub_directory(fn)
                    obj=addFromDirectory(obj,path_fn,pat,add_recursive);
                end
            elseif ~isempty(regexp(fn,pat_regexp,'once'))
                obj=addFromFile(obj,path_fn);
            end
        end
    else
        error('moxunit:illegalParameter','Input is not a directory');
    end



function tf=is_sub_directory(fn)
    tf=~any(strcmp(fn,{'.','..'}));
