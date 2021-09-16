function tf=moxunit_util_isfolder(folderName)
% return true if the argument is a folder
%
% This function is needed to keep compatibility with older versions of Octave and Matlab
% for which the function isfolder didn't exist yet.
%
% tf=moxunit_util_isfolder(folderName)
%
% Output:
%   tf               True if the argument points to an existing directory
%


    persistent cached_handle;

    if ~isempty(cached_handle)
        tf = cached_handle(folderName);
        return;
    end

    % We find out which function we need to redirect to
    v = moxunit_util_platform_version();
    isfolder_available = false;
    if moxunit_util_platform_is_octave()
        isfolder_available = (v(1) >= 5);
    else
        isfolder_available = (v(1) >= 10) || ((v(1) >= 9) && (v(2) >= 3));
    end

    % Call the appropriate function
    if isfolder_available
        cached_handle = @isfolder;
    else
        cached_handle = @isdir;
    end

    tf = cached_handle(folderName);

