function test_suite=test_moxunit_util_input2str
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;


function test_moxunit_util_input2str_basics
    aeq=@(expected,varargin)assertEqual(sprintf(expected{:}),...
                        moxunit_util_input2str(varargin{:}));
    w=randstr();
    x=randstr();
    y=randstr();
    z=randstr();
    ys=moxunit_util_elem2str(y);
    zs=moxunit_util_elem2str(z);

    aeq({'%s\n%s\n',w,x},w,x);
    aeq({'%s\n%s\n\nInput: %s\n',w,x,ys},w,x,y);
    aeq({'%s\n%s\n\nFirst input: %s\n\nSecond input: %s\n',...
                                        w,x,ys,zs},w,x,y,z);


function s=randstr()
    s=char(rand(1,10)*24+65);


