function tf=moxunit_util_is_test_function_name(name)
% Return true if the input is a string indicating a test function
%
% tf=moxunit_util_is_test_function_name(name)
%
% Input:
%   name                string
%
% Output:
%   tf                  true if name is a string and either starts with
%                       'test_' or ends with '_test'
%
% NNO Jan 2014

    tf=ischar(name) && numel(name)>=5 &&...
            (strcmp(name(1:5),'test_') || strcmp(name(end-(4:0)),'_test'));
