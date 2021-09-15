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

    s=sprintf('%s(%s,%s)>', class(obj), getName(obj), getLocation(obj));
