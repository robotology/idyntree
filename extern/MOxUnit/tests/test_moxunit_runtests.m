function test_suite=test_moxunit_runtests
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_moxunit_runtests_basics
    slow_flag = ispc() && moxunit_util_platform_is_octave();
    if slow_flag
        % Skip if running in octave on windows. From some reason Octave
        % chokes heavilly on temporary file access and deletion.
        reason = '''test_moxunit_runtests_basics'' is very slow in Octave on Windows!';
        moxunit_throw_test_skipped_exception(reason)
        fprintf('This test will take a very long time\n');
    end
    passed_cell={'','abs(2);','for k=1:10,2+k;end'};
    failed_cell={'error(''expected'');','[1,2]+[1,2,3];'};
    skipped_cell={'moxunit_throw_test_skipped_exception(''skip'')'};

    combis=all_binary_combinations(8);
    for k=1:size(combis,1)
        log_fn=tempname();
        cleanup_helper('add_file',log_fn);

        combi=combis(k,:);
        [has_passed,has_failed,has_skipped,...
                use_recursion,in_subdirectory,...
                has_error,verbose_output,add_suite_instance]=deal(combi{:});


        dir_to_test=tempname();
        cleanup_helper('add_directory',dir_to_test);
        mkdir_recursively(dir_to_test);

        if in_subdirectory
            % put tests in subdirectory. When not using recursive option in
            % moxunit_runtests, then no tests are found.
            dir_with_tests=tempname(dir_to_test);
            cleanup_helper('add_directory',dir_with_tests);
            mkdir_recursively(dir_with_tests);
        else
            dir_with_tests=dir_to_test;
        end

        % make directory for tests
        n_passed=add_tests(dir_with_tests,has_passed,passed_cell);
        n_failed=add_tests(dir_with_tests,has_failed,failed_cell);
        n_skipped=add_tests(dir_with_tests,has_skipped,skipped_cell);

        args={'-logfile',log_fn};
        if add_suite_instance
            suite=MOxUnitTestSuite();
            suite=addFromDirectory(suite,dir_to_test,...
                                    '.*\.m',use_recursion);
            args{end+1}=suite;
        else
            args{end+1}=dir_to_test;
        end

        if has_error
            args{end+1}='-foo';
        end

        if verbose_output
            args{end+1}='-verbose';
        end

        if use_recursion
            args{end+1}='-recursive';
        end

        handle=@()moxunit_runtests(args{:});
        if has_error
            assertExceptionThrown(handle,'moxunit:illegalParameter');
        else
            result=handle();

            if in_subdirectory && ~use_recursion
                test_stats=[0 0 0];
                must_have_passed=true;
            else
                test_stats=[n_passed,n_failed,n_skipped];
                must_have_passed=~(has_error || has_failed);
            end

            assertEqual(must_have_passed, result);

            test_labels={'.','F','s';...
                         'passed','failure','skipped'};
            assert_logfile_matches(log_fn,verbose_output,...
                                    test_stats,test_labels)

        end
        cleanup_helper('cleanup');
        if slow_flag
            fprintf('Combination test #%d/%d\n',k,size(combis,1))
        end
    end

function test_moxunit_runtests_partitions

    cleaner=onCleanup(@()cleanup_helper('cleanup'));
    dir_to_test=tempname();
    mkdir(dir_to_test);
    cleanup_helper('add_directory',dir_to_test);
    log_fn=tempname();
    cleanup_helper('add_file',log_fn);

    test_partition_count=3+ceil(rand()*5);
    ntests_per_partitions=3+ceil(rand()*5);

    ntests=test_partition_count*ntests_per_partitions;

    test_str_cell=cell(ntests,1);
    passes=false(ntests,1);
    for k=1:ntests
        p=rand()>.5;
        passes(k)=p;

        if p
            test_str='abs(2);';
        else
            test_str='error(''foo'')';
        end

        test_str_cell{k}=test_str;
    end

    add_tests(dir_to_test,true,test_str_cell);


    test_labels={'.','F'};
    for test_partition_index=1:test_partition_count
        args={dir_to_test,'-logfile',log_fn,...
                            '-partition_index',test_partition_index,...
                            '-partition_count',test_partition_count};

        result=moxunit_runtests(args{:});
        idx=test_partition_index:test_partition_count:ntests;
        use_passes=passes(idx);
        assertEqual(result,all(use_passes));

        test_stats=[sum(use_passes), sum(~use_passes), 0];
        assert_logfile_matches(log_fn,false,...
                                    test_stats,test_labels)
    end


function count=add_tests(test_dir,do_add,cell_with_tests)
    count=0;
    if do_add
        for k=1:numel(cell_with_tests)
            idx=cleanup_helper('count');

            name=sprintf('test_%03d',idx);
            fn=fullfile(test_dir,sprintf('%s.m',name));
            cleanup_helper('add_file',fn);
            write_test_mfile(fn,name,cell_with_tests{k});

            count=count+1;
        end
    end


function assert_logfile_matches(log_fn,verbose_output,...
                                test_stats,test_labels)
    fid=fopen(log_fn);
    cleaner=onCleanup(@()fclose(fid));

    content=fread(fid,'char=>char')';
    result_start=strfind(content,sprintf('\n'));
    result_end=strfind(content,'------');

    result=content(result_start(1):result_end(1));

    labels_row=1;
    if verbose_output
        labels_row=labels_row+1;
    end

    labels=test_labels(labels_row,:);
    n_labels=numel(labels);

    for k=1:n_labels
        pat=regexptranslate('escape',labels{k});
        count=numel(regexp(result,pat,'start'));
        assertEqual(count,test_stats(k));
    end


function combis=all_binary_combinations(count)
    n=2^(count-1);
    first_half=1:n;
    second_half=first_half+n;

    combis=cell(n,count);
    combis(first_half,1)=repmat({true},1,n);
    combis(second_half,1)=repmat({false},1,n);

    if count>1
        next_combi=all_binary_combinations(count-1);
        combis(first_half,2:end)=next_combi;
        combis(second_half,2:end)=next_combi;
    end


function write_test_mfile(fn,name,body)
    content=sprintf(['function test_suite=%s\n'...
                      'try\n'...
                      '    test_functions=localfunctions();\n',...
                      'catch\n',...
                      'end\n',...
                      'initTestSuite;\n',...
                     '\n'...
                     'function %s_func\n',...
                     '    %s'],...
                     name,name,body);

    % make sure directory exists
    parent=fileparts(fn);
    mkdir_recursively(parent);

    fid=fopen(fn,'w');
    file_closer=onCleanup(@()fclose(fid));
    fprintf(fid,'%s',content);

function mkdir_recursively(dir_name)
    if ~exist(dir_name,'dir')
        parent=fileparts(dir_name);
        mkdir_recursively(parent);
        mkdir(dir_name);
    end


function c=cleanup_helper(task,varargin)
%   cleanup_helper('add_file','foo')
%   cleanup_helper('add_directory','bar')
% add the file or directory 'foo' or 'bar' to the list
%   of files and directories stored internally
% cleanup_helper('cleanup') removes all internally stores files and
%   directory
%
    persistent directories;
    persistent files;

    c=0;

    switch task
        case 'add_file'
            if isnumeric(files)
                files=cell(0);
            end

            files{end+1}=varargin{1};

        case 'add_directory'
            assert(numel(varargin)==1);
            if isnumeric(directories)
                directories=cell(0);
            end

            directories{end+1}=varargin{1};

        case 'cleanup'
            assert(numel(varargin)==0);
            if iscell(files)

                if moxunit_util_platform_is_octave()
                    for k=1:numel(files)
                        file = files{k};
                        if exist(file,'file')
                            %tic
                            unlink(file); % This is a bit faster in Octave
                            %toc
                        end
                    end

                else
                    s = warning('error','MATLAB:DELETE:Permission'); %#ok<CTPCT>
                    c = onCleanup(@() warning(s)); % Reset warning state

                    for k=1:numel(files)
                        file = files{k};
                        if exist(file,'file')
                            try %#ok<TRYNC>
                                delete(file);
                            end
                        end
                    end
                end

                files=[];
            end

            if iscell(directories)
                if moxunit_util_platform_is_octave()
                    % GNU Octave requires, by defaualt, confirmation when
                    % using rmdir - unless confirm_recursive_rmdir is set
                    % explicitly
                    % Here the state of confirm_recursive_rmdir is stored,
                    % and set back to its original value when leaving this
                    % function.
                    confirm_val=confirm_recursive_rmdir(false);
                    cleaner=onCleanup(@()confirm_recursive_rmdir(confirm_val));
                end

                for k=1:numel(directories)
                    directory=directories{k};
                    if moxunit_util_isfolder(directory)
                        rmdir(directory,'s');
                    end
                end

                directories=[];
            end

        case 'count'
            c=numel(directories)+numel(files);

        otherwise
            assert(false,'illegal task')
    end
