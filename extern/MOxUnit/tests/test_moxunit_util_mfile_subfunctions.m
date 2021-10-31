function test_suite=test_moxunit_util_mfile_subfunctions
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function s=rand_str()
    s=char(20*rand(1,10)+65);

function test_multiple_subfunctions
    % variable number of functions in body
    for func_count=0:5
        func_names=cell(1,func_count);
        lines=cell(func_count*2,1);

        for k=1:func_count
            func_name=rand_str();
            func_names{k}=func_name;

            lines{k*2-1}=[rand_str() rand_str()];
            lines{k*2}=sprintf('function %s',func_name);
        end

        n_out=zeros(1,func_count);
        helper_test_with_lines(func_names, n_out, lines);
    end

function test_multiple_subfunctions_nout
    % variable number of functions in body, each with variable number of
    % output arguments
    argouts={{'','   '},...
             {'~','[~]','a','aa','[a]','  [ aabb ] '},...
             {'[~, aa]','[~ ~]','[a bb ]','[a,bb]','[ ab ,  ba ]'},...
             {'[~,~,~]',' [ a, bb, c ]','[  a,~,b ]'},...
             };

    for func_count=0:10
        func_names=cell(1,func_count);
        lines=cell(func_count*2,1);

        n_out=zeros(func_count,1);
        for k=1:func_count
            func_name=rand_str();
            func_names{k}=func_name;

            n_out_func_plus_one=ceil(rand()*numel(argouts));
            argout_cell=argouts{n_out_func_plus_one};
            argout_str=argout_cell{ceil(rand()*numel(argout_cell))};

            if n_out_func_plus_one>1
                argout_str=sprintf('%s = ',argout_str);
            end

            lines{k*2-1}=[rand_str() rand_str()];
            lines{k*2}=sprintf('function %s %s',argout_str,func_name);

            n_out(k)=n_out_func_plus_one-1;
        end

        helper_test_with_lines(func_names, n_out, lines);
    end


function test_commented_subfunction()
    helper_test_with_lines({},[],{',','% function foo',''});

function test_in_string_subfunction()
    helper_test_with_lines({},[],{'','''function foo''',''});

function test_no_whitespace_subfunction()
    helper_test_with_lines({},[],{'functionfoo'});

function test_newline_subfunction()
    helper_test_with_lines({'foo'},0,{sprintf('function ...\nfoo')});

function test_no_whitespace_newline_subfunction()
    helper_test_with_lines({},0,{sprintf('function...\nfoo')});

function test_tilde_newline_subfunction()
    helper_test_with_lines({'foo'},1,{sprintf('function ~=foo')});

function test_multiple_tilde_newline_subfunction()
    helper_test_with_lines({'foo'},2,{sprintf('function[~,~]=foo')});

function test_no_space_subfunction_long_vars
    helper_test_with_lines({'foo'},1,{sprintf('function aa=foo(bb, cc)')});

function test_no_space_subfunction_short_vars
    helper_test_with_lines({'foo'},1,{sprintf('function a=foo(b, c)')});

function test_different_args_subfunction
    % test with various types of whitespace, number of input arguments, and
    % number of output arguments

    slow_flag = ispc() && moxunit_util_platform_is_octave();
    if slow_flag
        % Skip if running in octave on windows. From some reason Octave
        % chokes heavilly on temporary file access and deletion.
        reason = '''test_different_args_subfunction'' is very slow in Octave on Windows!';
        moxunit_throw_test_skipped_exception(reason)
        fprintf('This test will take a very long time\n');
    end

    whitespace_cell={' ',...
                     '  ',...
                     sprintf('\t'),...
                     sprintf(' ...\n'),...
                     sprintf(' ...\r\n')};
    for i_sp=1:numel(whitespace_cell)
        whitespace=whitespace_cell{i_sp};
        for n_out=-1:3
            arg_out=helper_build_arg_list(n_out,whitespace,'[',']','=');
            for n_in=-1:3
                arg_in=helper_build_arg_list(n_in,whitespace,'(',')','');
                func_name=rand_str();
                parts={'function',arg_out,func_name,arg_in};
                line=moxunit_util_strjoin(parts,whitespace);
                helper_test_with_lines({func_name},max(n_out,0),{line});
                if slow_flag
                    fprintf('i_sp: %0f, n_out: %0f, n_in: %0f\n',i_sp,n_out,n_in)
                end
            end
        end
    end

function arg_list=helper_build_arg_list(n_args, whitespace, ...
                                left_paren, right_paren, suffix)
    if n_args<0
        arg_list='';
    else
        param_names=arrayfun(@(unused)rand_str,ones(1,n_args),...
                            'UniformOUtput',false);
        delim=[whitespace ',' whitespace];
        args=moxunit_util_strjoin(param_names,delim);
        arg_list_cell={left_paren,args,right_paren,suffix};
        arg_list=moxunit_util_strjoin(arg_list_cell, whitespace);
    end


function helper_test_with_lines(func_names, n_out, lines)
% func_names is expected cell output from
% moxunit_util_mfile_subfunctions
% when applied to a file containing the 'lines' data
%
% nout is a struct with a field nargout containing
% the number of output arguments for each function
    assert(iscell(func_names));
    assert(iscell(lines));

    tmp_fn=tempname();
    if moxunit_util_platform_is_octave
        cleaner=onCleanup(@()unlink(tmp_fn)); % Faster in Octave
    else
        cleaner=onCleanup(@()delete(tmp_fn));
    end

    % try different line endings
    line_ending_cell={'\r\n',...  % MS Windows
                        '\n'};     % Unix-like / OSX

    header_to_ignore=['function ' rand_str];
    lines_with_header=[{header_to_ignore}; lines(:)];

    for k=1:numel(line_ending_cell)
        write_text_file(tmp_fn,lines_with_header,line_ending_cell{k});
        fs=moxunit_util_mfile_subfunctions(tmp_fn);

        % must be a struct
        assert(isstruct(fs));

        % number of functions should match
        n_func=numel(func_names);
        assertEqual(n_func,numel(fs));

        % verify content of each element
        for j=1:n_func
            f=fs(j);
            assert(isfield(f,'name'));
            assertEqual(func_names{j},f.name);

            assert(isfield(f,'nargout'));
            assertEqual(n_out(j),f.nargout);
        end
    end


function fn=write_text_file(fn, lines, line_ending)
    fid=fopen(fn,'w');
    fprintf(fid,['%s' line_ending],lines{:});
    fclose(fid);
