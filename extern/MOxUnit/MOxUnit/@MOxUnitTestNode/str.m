function s=str(obj)
% Return string representation of MoxUnitTestNode instance
%
% s=str(obj)
%
% Input:
%   obj             empty MOxUnitTestNode instance.
%
% Output:
%   s               string representation of obj.
%
% NNO 2015

    s=sprintf('<abstract %s object>', class(obj));
