function string=moxunit_util_input2str(message,whatswrong,a,b)
% Convert input to string representation
%
% string=moxunit_util_input2str(message,whatswrong[,a[,b]])
%
% Inputs:
%   message         string with custom message, or empty string
%   whatswrong      string indicating an error
%   a               optional first element  } of any
%   b               optional second element } type
%
% Returns:
%   string          string representation of the inputs a and b (if
%                   provided), prefixed by whatswrong, prefixed by message
%                   (if message is not empty)
%
% Notes:
%   - This is a helper function for assertElementsAlmostEqual,
%     assertVectorsAlmostEqual, assertEqual, assertTrue, assertFalse
%
% See also: assertElementsAlmostEqual, assertVectorsAlmostEqual,
%           assertEqual, assertTrue, assertFalse
%
% NNO Jan 2014

    if isempty(message)
        prefix=whatswrong;
    else
        prefix=sprintf('%s\n%s',message,whatswrong);
    end

    if nargin<3
        % no elements
        string=sprintf('%s\n',prefix);

    elseif nargin<4
        % one element
        string=sprintf('%s\n\nInput: %s\n', ...
                        prefix,moxunit_util_elem2str(a));

    else
        % two elements
        string=sprintf('%s\n\nFirst input: %s\n\nSecond input: %s\n',...
                        prefix, moxunit_util_elem2str(a),...
                                moxunit_util_elem2str(b));
    end
