function assertNotEqual(a, b, message)
% assert that two inputs are not equal
%
% assertNotEqual(a,b,[msg])
%
% Inputs:
%   a           first input  } of any
%   b           second input } type
%   msg             optional custom message
%
% Raises:
%   'assertNotEqual:equal'                a and b are equal, and are either
%                                         both sparse or both not sparse
%
% Examples:
%   assertNotEqual('foo','bar');
%   %|| % passes without output
%
%   assertNotEqual(1,sparse(1));
%   %|| % passes without output
%
%   assertNotEqual('foo','foo');
%   %|| error('elements are equal');
%
% Notes:
%   - If a custom message is provided, then any error message is prefixed
%     by this custom message
%   - In this function, NaN values are considered equal.
%   - This function attempts to show similar behaviour as in
%     Steve Eddins' MATLAB xUnit Test Framework (2009-2012)
%     URL: http://www.mathworks.com/matlabcentral/fileexchange/
%                           22846-matlab-xunit-test-framework
%

    % Note: although it may seem more logical to compare class before size,
    % for compatibility reasons the order of tests matches that of the
    % MATLAB xUnit framework

    if isequaln_wrapper(a,b) && issparse(a)==issparse(b)
        whatswrong='inputs are equal';
        error_id='assertNotEqual:equal';

    else
        % elements are equal
        return;
    end

    if nargin<3
        message='';
    end

    full_message=moxunit_util_input2str(message,whatswrong,a,b);

    if moxunit_util_platform_is_octave()
        error(error_id,'%s',full_message);
    else
        throwAsCaller(MException(error_id, '%s', full_message));
    end

function tf=isequaln_wrapper(a,b)
% wrapper to support old versions of Matlab
    persistent has_equaln;

    if isempty(has_equaln)
        has_equaln=~isempty(which('isequaln'));
    end

    if has_equaln
        tf=isequaln(a,b);
    else
        tf=isequalwithequalnans(a,b);
    end
