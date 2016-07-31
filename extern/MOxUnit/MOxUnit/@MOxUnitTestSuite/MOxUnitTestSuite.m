function obj=MOxUnitTestSuite(name)
% Initialize empty test suite
%
% Input:
%   name            Optional name of the test suite
% Output:
%   obj             MoxUnitTestSuite instance with no tests.
%
% Notes:
%   MOxUnitTestSuite is a subclass of MoxUnitTestNode.
%
% See also: MoxUnitTestNode
%
% NNO 2015

    class_name='MOxUnitTestSuite';
    if nargin<1
        name=class_name;
    end

    s=struct();
    s.tests=cell(0);
    obj=class(s,class_name,MOxUnitTestNode(name));


