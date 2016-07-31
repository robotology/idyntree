function s=str(obj)
% Return string representation of MoxUnitTestCase instance
%
% s=str(obj)
%
% Input:
%   obj             MoxUnitTestCase instance.
%
% Output:
%   s               string representation of obj.
%
    s=sprintf('<abstract %s object>', class(obj));
