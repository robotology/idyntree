function obj=MOxUnitTestCase(name, location)
% Initialize empty test case
%
% Input:
%   name            Name of the test case
%   location        Location of the test case
% Output:
%   obj             abstract MOxUnitTestCase instance
%
% Notes:
%   - MOxUnitFunctionHandleTestCase is a subclass of MOxUnitTestCase, which
%     in turn is a subclass of MOxUnitTestNode.
%
% See also: MoxUnitTestNode, MOxUnitFunctionHandleTestCase

    s=struct();
    s.location=location;
    obj=class(s,'MOxUnitTestCase',MOxUnitTestNode(name));

