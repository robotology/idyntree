function assertGreaterThan(a, b, message)
% assert that a is greater than b
%
% assertGreaterThan(a,b,[msg])
%
% Inputs:
%   a           first input  } of any
%   b           second input } type
%   msg             optional custom message
%
% Raises:
%   'assertGreaterThan:sizeNotEqual'         a and b are of different size
%   'assertGreaterThan:classNotEqual'        a and b are of different class
%   'assertGreaterThan:notGreaterThan'       values in a must be greater
%                                            than values in b
%
% Examples:
%   assertGreaterThan(2,1);
%   %|| % passes without output
%
%   assertGreaterThan(1,2);
%   %|| error('first input argument in not larger than the second');
%
%   assertGreaterThan([2 3],[0;1]);
%   %|| error('inputs are not of the same size');

    if ~isequal(size(a), size(b))
        whatswrong='inputs are not of the same size';
        error_id='assertGreaterThan:sizeNotEqual';

    elseif ~isequal(class(a), class(b))
        whatswrong='inputs are not of the same class';
        error_id='assertGreaterThan:classNotEqual';

    elseif length(a) == 1 && length(b) == 1 && ~(a > b)
        whatswrong='first input argument in not larger than the second';
        error_id='assertGreaterThan:notGreaterThan';

    elseif ~all(a > b)
        whatswrong=['each element in the first input must be greater than ',...
                    'the corresponding element in the second input'];
        error_id='assertGreaterThan:notGreaterThan';

    else
        % assertion passed
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