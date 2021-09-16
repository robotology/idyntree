function test_suite=test_moxunit_util_elem2str
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_moxunit_util_elem2str_tiny
    aeq=@assert_expected_output;
    % empty string
    aeq('0x0 char (empty)\n''''','');

    % string in row vector form
    aeq('1x3 char\n''abc''','abc');

    % matrix
    aeq('2x3 double\n[4 3 2;3 4 1]',[4 3 2; 3 4 1]);

    % random data
    x=randn(2);
    precision=5;
    aeq(['2x2 double\n' mat2str(x,precision)],x);

function test_moxunit_util_elem2str_big_matrix
    aeq=@assert_expected_output;

    aeq(        ['4x4 double\n'...
                 '[1 -1e-08 -1e-08 -1e-08;'...
                 '-1e-08 1 -1e-08 -1e-08;'...
                 '-1e-08 -1e-08 1 -1e-08;'...
                 '-1e-08 -1e-08 -1e-08 1]'...
                 ],...
                 eye(4)-1e-8);

function test_moxunit_util_elem2str_big_cell
    aeq=@assert_expected_output;

    aeq('100x2 cell',cell(100,2))

function test_moxunit_util_elem2str_tiny_with_evalc
    aeq=@assert_expected_output_evalc_if_present;

    % string with non-row vector form
    aeq('2x3 char\nabc\ndef',...
        '2x3 char',...
            ['abc';'def']);

    % 3D string array
    aeq('2x3x2 char\n\n(:,:,1) =\n\nabc\ncde\n\n(:,:,2) =\n\nefg\nghi',...
                '2x3x2 char',...
                cat(3,['abc';'cde'],['efg';'ghi']))

    % logical array
    aeq('1x2 logical\n     0     1','1x2 logical',[false true]);

    % cell array
    aeq({'1x2 cell\n    ''foo''    [1]',...
        '1x2 cell\n\n{\n[1,1] = foo\n[1,2] =  1\n}',...
        '1x2 cell\n    {''foo''}    {[1]}'},...
        '1x2 cell',{'foo',1});


function test_moxunit_util_elem2str_custom_class
% Make a temporary class with a size function that throws an error;
% This tests the functionality of util_elem2str for the case that not
% even a class' 'size' function is available
    tmpdir=tempname(tempdir);
    cleaner=onCleanup(@()remove_path_directory(tmpdir));

    classname=sprintf('my_class');
    classdir=fullfile(tmpdir,sprintf('@%s',classname));

    if ~exist(tmpdir,'dir')
        % GNU Octave returns a temporary directory that does not exist;
        % so create it first
        mkdir(tmpdir);
    end

    % make subdirectory for the class
    mkdir(classdir);

    write_contents(classdir,classname,...
                    ['function obj=%s()\n'...
                            'obj=class(struct(),''%s'');'],...
                            classname,classname);
    write_contents(classdir,'size',...
                    ['function size(obj)\n'...
                            'error(''raises error'');']);
    addpath(tmpdir);
    constructor=str2func(classname);

    % make object instance
    obj=constructor();

    aeq=@assert_expected_output;
    aeq(sprintf('%s',classname),obj);

function write_contents(dirname,fname,pat,varargin)
% helper function to write .m file
    fn=fullfile(dirname,sprintf('%s.m',fname));

    fid=fopen(fn,'w');
    cleaner=onCleanup(@()fclose(fid));
    fprintf(fid,pat,varargin{:});


function remove_path_directory(dir_name)
    % removes dir_name from the search path and from the file system

    rmpath(dir_name);
    if moxunit_util_platform_is_octave()
        % GNU Octave requires, by defaualt, confirmation when using rmdir.
        % The state of confirm_recursive_rmdir is stored, and set back
        % to its original value when leaving this function.
        confirm_val=confirm_recursive_rmdir(false);
        cleaner=onCleanup(@()confirm_recursive_rmdir(confirm_val));
    end

    rmdir(dir_name,'s');


function assert_expected_output_evalc_if_present(a,b,varargin)
    mex_file_code=3;
    if exist('evalc','builtin') || ...
            exist('evalc')==mex_file_code
        to_compare=a;
    else
        to_compare=b;
    end

    assert_expected_output(to_compare,varargin{:});


function assert_expected_output(to_compare,varargin)
    result=moxunit_util_elem2str(varargin{:});

    if islogical(varargin{1})
        assert(ischar(to_compare));
        expected=sprintf(to_compare);
        if ~is_equal_modulo_whitespace(result,expected);
            assertEqual(to_compare,expected,'Not equal modulo whitespace');
        end
    else
        if ~iscell(to_compare)
            to_compare={to_compare};
        end

        expected_cell=cellfun(@sprintf,to_compare,'UniformOutput',false);

        for k=1:numel(expected_cell)
            if is_equal_modulo_whitespace(expected_cell{k},result)
                return;
            end
        end

        msg=sprintf(['Output ''%s'' is not equal modulo whitespace '...
                    'to any of: ''%s'''],...
                     result,...
                     moxunit_util_strjoin(expected_cell,''', '''));
        error(msg);
    end


function tf=is_equal_modulo_whitespace(a,b)
    simplify_whitespace=@(x)regexprep(x,'\s+',' ');
    tf=isequal(simplify_whitespace(a),simplify_whitespace(b));


