function obj=MOxUnitFunctionHandleTestCase(name,location,function_handle)
% Initialize MOxUnitFunctionHandleTestCase instance
%
% obj=MOxUnitFunctionHandleTestCase(name, location, function_handle)
%
% Inputs:
%   name            String with descriptive name of function_handle
%   location        String with location of function_handle
%   function_handle Function handle that, when called through:
%
%                       function_handle()
%
%                   raises an exception if the test associated with the
%                   handle fails.
%
% Output:
%   obj             MOxUnitFunctionHandleTestCase instance.
%
% NNO 2015

    s=struct();
    s.function_handle=function_handle;

    obj=class(s,'MOxUnitFunctionHandleTestCase',...
                    MOxUnitTestCase(name,location));


