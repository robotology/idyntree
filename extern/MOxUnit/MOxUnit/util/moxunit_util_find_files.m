function result=moxunit_util_find_files(root_dir,re,add_recursive)
% Find files matching a pattern
%
% result=moxunit_util_find_files(root_dir,re,add_recursive)
%
% Inputs:
%   root_dir        Root directory in which to look for files.
%   re              Regular expression of file names to match
%   add_recursive   (optional) If true, then files are added recursively
%                   from sub-directories of root_dir (and from sub-sub-,
%                   sub-sub-sub-,... -directories). If false then only
%                   files in root_dir, but not in its subdirectories, are
%                   considered. Default: true
%
% Output:
%   result          Cell with matching files in directory
%
% NNO 2015

    if ~moxunit_util_isfolder(root_dir)
        error('Diectory input argument must be a directory');
    end

    if ~ischar(re)
        error('Second argument must be a string');
    end

    if nargin<3 || isempty(add_recursive)
        add_recursive=true;
    end

    result=find_files_helper(root_dir,re,add_recursive);


function result=find_files_helper(root_dir,re,add_recursive)
    if moxunit_util_isfolder(root_dir)
        d=dir(root_dir);
        n=numel(d);

        result_cell=cell(n,1);

        for k=1:n
            fn=d(k).name;

            if ignore_directory(fn)
                continue;
            end

            path_fn=fullfile(root_dir,fn);

            if moxunit_util_isfolder(path_fn)
                if add_recursive
                    result_cell{k}=find_files_helper(path_fn,re,...
                                                        add_recursive);
                end

            elseif moxunit_util_regexp_matches(fn,re)
                result_cell{k}={path_fn};
            end
        end

        keep=~cellfun(@isempty,result_cell);
        result_cell=result_cell(keep);

        result=cat(1,result_cell{:});
    end

function tf=ignore_directory(fn)
    tf=any(strcmp(fn,{'.','..'}));
