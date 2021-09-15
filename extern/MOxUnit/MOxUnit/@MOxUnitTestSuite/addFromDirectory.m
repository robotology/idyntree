function obj=addFromDirectory(obj,directory,pat,add_recursive)
% Add unit tests from directory
%
% obj=addFromFile(obj,directory[,pat])
%
% Inputs:
%   obj             MoxUnitTestSuite instance.
%   directory       name of directory that may contain files with top-level
%                   function that returns MOxUnitTestNode instances.
%   pattern         Regular expression containing file pattern of files to
%                   add.
%   add_recursive   If true, then files are added recursively from
%                   sub-directories. If false (the default) then only files
%                   are added from directory, but files in subdirectories
%                   are ignored.
%
% Output:
%   obj             MoxUnitTestSuite instance with MOxUnitTestNode
%                   instances  added, if present in the files in directory.
%
% See also: initTestSuite, addFromFile
%
% NNO 2015

    if nargin<4 || isempty(add_recursive)
        add_recursive=false;
    end

    % find the files
    filenames=moxunit_util_find_files(directory,pat,add_recursive);

    n_filenames=numel(filenames);

    for k=1:n_filenames
        filename=filenames{k};
        obj=addFromFile(obj,filename);
    end
