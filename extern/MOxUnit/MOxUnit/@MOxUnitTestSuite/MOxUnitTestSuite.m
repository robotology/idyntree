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
    s.test_count=0;
    s.tests=cell(10,1); % some space to start with
    obj=class(s,class_name,MOxUnitTestNode(name));


