function directories_added=moxunit_set_path()
% sets the search path for MOxUnit
%
% directories_added=moxunit_set_path()
%
% Output:
%    directories_added          Cell string with directories that were
%                               added to the search path. If no
%                               directories were added to the search path
%                               (i.e., all functions for MOxUnit were
%                               already in the search path), then this is
%                               an empty cell array.
% Note:
%   - This adds directory in which this function resides, as well as the
%     'util' sub-directory to the search path

    return_directories_added=nargout>=1;

    root_dir=fileparts(mfilename('fullpath'));

    sub_dirs={'','util'};

    % Cannot use moxunit_util_strjoin because it may not be in the path.
    % Instead construct the path to be added by adding the pathsep() after
    % each subdirectory
    make_full_path_with_sep=@(sub_dir)[fullfile(root_dir,sub_dir) ...
                                        pathsep()];

    full_dirs=cellfun(make_full_path_with_sep,sub_dirs,...
                        'UniformOutput',false);

    if return_directories_added
        % store original path elements
        orig_path_cell=get_path_cell();
    end

    addpath(cat(2,full_dirs{:}));

    if return_directories_added
        new_path_cell=get_path_cell();
        directories_added=setdiff(new_path_cell, orig_path_cell);
    end

function pc=get_path_cell()
    pc=regexp(path(), pathsep(), 'split');

