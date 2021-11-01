function flag=moxunit_util_platform_supports(key)
% Return whether the current Matlab / octave supports certain functionality
%
% flag=moxunit_util_platform_supports(key)
%
% Inputs:
%   key     Functionality, must be one of:
%           'localfunctions_in_script'  - whether scripts called from a
%                                         function have access to local
%                                         subfunctions in that function
%                                         (Functionality removed in Matlab
%                                         2016a)
%           'diagnostics_recording_plugin' - whether the
%                                         matlab.unittest.plugins
%                                           .DiagnosticsRecordingPlugin
%                                         is available
%
% Output:
%   flag    true or false, indicating whether the current platform supports
%           the functionality indicated by the key argument

    if ~ischar(key)
        error('Input must be a string');
    end

    switch key
        case 'localfunctions_in_script'
            if moxunit_util_platform_is_octave()
                v=moxunit_util_platform_version();
                flag=v(1)<6; % before 6.0.0
            else
                v=moxunit_util_platform_version();
                flag=v(1)<9; % before 2016a
            end

        case 'diagnostics_recording_plugin'
            s='matlab.unittest.plugins.DiagnosticsRecordingPlugin';
            flag=~isempty(which(s));

        otherwise
            error('Unsupported key %s', key);
    end
