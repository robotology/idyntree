function s=str(obj)
% return string representation of MoxUnitFunctionHandleTestCase object
%
% s=str(obj)
%
% Inputs:
%   obj             MoxUnitFunctionHandleTestCase object
%
% Output:
%   s               String representation showing the name and location of
%                   obj.
%
% NNO 2015-2015

    s=sprintf('%s:  %s', getName(obj), getLocation(obj));
