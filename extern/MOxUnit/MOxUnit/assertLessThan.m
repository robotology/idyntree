function assertLessThan(a, b, message)
% assert that a is less than b
%
% assertLessThan(a,b,[msg])
%
% Inputs:
%   a           first input  } of any
%   b           second input } type
%   msg             optional custom message
%
% Raises:
%   'assertLessThan:sizeNotEqual'         a and b are of different size
%   'assertLessThan:classNotEqual'        a and b are of different class
%   'assertLessThan:notLessThan'          values in a must be less
%                                         than values in b
%
% Examples:
%   assertLessThan(1,2);
%   %|| % passes without output
%
%   assertLessThan(2,1);
%   %|| error('first input argument in not smaller than the second');
%
%   assertLessThan([0 1],[2;3]);
%   %|| error('inputs are not of the same size');

    if ~isequal(size(a), size(b))
        whatswrong='inputs are not of the same size';
        error_id='assertLessThan:sizeNotEqual';

    elseif ~isequal(class(a), class(b))
        whatswrong='inputs are not of the same class';
        error_id='assertLessThan:classNotEqual';

    elseif length(a) == 1 && length(b) == 1 && ~(a < b)
        whatswrong='first input argument in not smaller than the second';
        error_id='assertLessThan:notLessThan';

    elseif ~all(a < b)
        whatswrong=['each element in the first input must be less than ',...
                    'the corresponding element in the second input'];
        error_id='assertLessThan:notLessThan';

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