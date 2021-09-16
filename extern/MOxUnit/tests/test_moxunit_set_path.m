function test_suite=test_moxunit_set_path
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_moxunit_set_path_root
    helper_test_with_subdir('');

function test_moxunit_set_path_util
    helper_test_with_subdir('util');

function helper_test_with_subdir(subdir)
    orig_path=path();
    path_resetter=onCleanup(@()path(orig_path));

    func=@moxunit_set_path;
    func_name=func2str(func);

    % remove from path
    root_dir=fileparts(which(func_name));
    relative_dir=fullfile(root_dir,subdir);
    rmpath(relative_dir);

    % should not be in path
    assert(~is_elem(path(),relative_dir,pathsep()));

    % must be now in path
    directories_added_cell=func();
    assert(is_elem(path(),relative_dir,pathsep()));

    % directory must have been added and part of the output
    assert(numel(directories_added_cell)>0)
    directories_added_str=sprintf(['%s' pathsep()],...
                            directories_added_cell{:});
    assert(is_elem(directories_added_str,relative_dir,pathsep()));


function tf=is_elem(haystack, needle, sep)
    tf=~isempty(strfind([sep haystack sep], [sep needle sep]));
