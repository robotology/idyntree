function assertEqual(a, b, message)
% assert that two inputs are equal
%
% assertEqual(a,b,[msg])
%
% Inputs:
%   a           first input  } of any
%   b           second input } type
%   msg             optional custom message
%
% Raises:
%   'moxunit:differentSize'         a and b are of different size
%   'moxunit:differentClass         a and b are of different class
%   'moxunit:differentSparsity'     a is sparse and b is not, or
%                                         vice versa
%   'moxunit:elementsNotEqual'      values in a and b are not equal
%
% Notes:
%   - If a custom message is provided, then any error message is prefixed
%     by this custom message
%   - This function attempts to show similar behaviour as in
%     Steve Eddins' MATLAB xUnit Test Framework (2009-2012)
%     URL: http://www.mathworks.com/matlabcentral/fileexchange/
%                           22846-matlab-xunit-test-framework
%
% NNO Jan 2014

    % Note: although it may seem more logical to compare class before size,
    % for compatibility reasons the order of tests matches that of the
    % MATLAB xUnit framework

    if ~isequal(size(a), size(b))
        whatswrong='inputs are not of the same size';
        error_id='assertEqual:nonEqual';

    elseif ~isequal(class(a), class(b))
        whatswrong='inputs are not of the same class';
        error_id='assertEqual:classNotEqual';

    elseif issparse(a)~=issparse(b)
        whatswrong='inputs do not have the same sparsity';
        error_id='assertEqual:sparsityNotEqual';

    elseif ~isequaln_wrapper(a, b)
        whatswrong='elements are not equal';
        error_id='assertEqual:nonEqual';

    else
        % elements are equal
        return;
    end

    if nargin<3
        message='';
    end

    full_message=moxunit_util_input2str(message,whatswrong,a,b);

    if moxunit_util_platform_is_octave()
        error(error_id,full_message);
    else
        throwAsCaller(MException(error_id, full_message));
    end

function tf=isequaln_wrapper(a,b)
% wrapper to support old versions of Matlab
    persistent has_equaln;

    if isequal(has_equaln,true)
        tf=isequaln(a,b);
    elseif isequal(has_equaln,false)
        tf=isequalwithequalnans(a,b);
    else
        has_equaln=~isempty(which('isequaln'));
        tf=isequaln_wrapper(a,b);
    end
