function test_suite = test_moxunit_util_strjoin
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_strjoin_basics
    aeq=@(a,varargin)assertEqual(moxunit_util_strjoin(varargin{:}),a);

    aeq('a b cc',{'a','b','cc'});
    aeq('a>#<b>#<cc',{'a','b','cc'}, '>#<');
    aeq(sprintf('a\tb\tcc'),{'a','b','cc'}, '\t');
    aeq('a\b\cc',{'a','b','cc'}, '\\');
    aeq('a+b=cc',{'a','b','cc'}, {'+','='});

function test_strjoin_exceptions
    aet=@(varargin)assertExceptionThrown(@()...
                        moxunit_util_strjoin(varargin{:}),'');

    % first argument must be cell with strings
    aet('a');
    aet(struct)
    aet(2);
    aet([]);
    aet({2});

    % second argument must be string or cell with strings
    aet({'a','b','c'},2);
    aet({'a','b','c'},struct);
    aet({'a','b','c'},[]);
    aet({'a','b','c'},[]);
    aet({'a','b','c'},{'x'});
    aet({'a','b','c'},{1,2});

