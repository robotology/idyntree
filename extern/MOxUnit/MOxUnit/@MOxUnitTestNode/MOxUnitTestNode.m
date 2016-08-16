function obj=MOxUnitTestNode(name)
% Initialize MOxUnitTestNode instance
%
% obj=MOxUnitTestNode()
%
% Input:
%   name            name of this TestNode
%
% Output:
%   obj             MOxUnitTestNode instance
%
% Notes:
%   - This class is intended as an 'abstract' superclass, and should not
%     be initialized in typical use cases. Instead, use its subclasses,
%     MoxUnitTestSuite and MoxUnitTestCase.
%
% NNO 2015

    s=struct();
    s.name=name;
    obj=class(s,'MOxUnitTestNode');
