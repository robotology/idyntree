function re=moxunit_util_get_test_name_regexp()
% return regular expression indicating a test name or function
%
% re=moxunit_util_get_test_name_regexp()
%
% Output:
%   re        regular expression that matches a string starting with 'test'
%             or ending with 'test' (case-insensitive)

    test_re='[Tt][Ee][sS][tT]';
    re=sprintf('^((%s.*)|(.*%s))$',test_re,test_re);

