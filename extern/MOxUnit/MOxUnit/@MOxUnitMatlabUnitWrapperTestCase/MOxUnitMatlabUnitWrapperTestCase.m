function obj=MOxUnitMatlabUnitWrapperTestCase(matlab_unittest_test_obj)
% instantiates wrapper around matlab.unittest.Test class
%
% obj=MOxUnitMatlabUnitWrapperTestCase(matlab_unittest_test_obj)
%
% Inputs:
%   matlab_unittest_test_obj    singleton matlab.unittest.Test object
%
% Output:
%   obj                         MOxUnitMatlabUnitWrapperTestCase instance
%                               containing the test
    if ~(isa(matlab_unittest_test_obj, 'matlab.unittest.Test') && ...
            numel(matlab_unittest_test_obj)==1)
        error('Input must be singleton matlab.unittest.Test class');
    end

    name=matlab_unittest_test_obj.Name;
    location=name;

    s=struct();
    s.matlab_unittest_test_obj=matlab_unittest_test_obj;

    obj=class(s,'MOxUnitMatlabUnitWrapperTestCase',...
                        MOxUnitTestCase(name,location));
