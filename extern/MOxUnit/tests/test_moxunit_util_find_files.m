function test_suite=test_moxunit_util_find_files()
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function s=randstr(n)
    if nargin<1
        n=10;
    end
    s=char(ceil(rand(1,n)*26+64));

function varargout=make_randstrs(suffix)
    n=nargout;
    varargout=arrayfun(@(unused)[randstr() suffix],ones(1,n),...
                            'UniformOutput',false);


function test_moxunit_util_find_files_basics()
    ext=['.' randstr(1)];

    while true
        [s1,s2,s3]=make_randstrs('');
        [m1,m2,m3,m4,m5,m6]=make_randstrs(ext);

        all_fns={s1,s2,s3,...
                 m1,m2,m3,m4,m5,m6};

        all_fns_are_different=numel(unique(all_fns))==numel(all_fns);
        if all_fns_are_different
            break;
        end
    end

    to_add={{m1},{m2},...
            {s1},{s1,m3},{s1,m4},...
            {s1,s2},{s1,s2,m5},...
            {s1,s2,s3},{s1,s2,s3,m6}};

    root_dir=tempname();
    full_paths=make_files(root_dir,to_add,ext);
    cleaner=onCleanup(@()remove_paths(full_paths));

    for is_recursive=[false,true]
        for with_pat=[false,true]
            if is_recursive
                if with_pat
                    pat=sprintf('(.%s)|(foo)|(%s)',m3(2:end),m6);
                    expected={{s1,m3},{s1,s2,s3,m6}};
                else
                    pat='.*';
                    msk=cellfun(@(x)strcmp(x{end}(end+[-1,0]),ext),to_add);
                    expected=to_add(msk);
                end
            else
                if with_pat
                    pat=sprintf('(%s.)|(%s)',m2(1:(end-1)),m3);
                    expected={{m2}};
                else
                    pat='.*';
                    expected={{m1},{m2}};
                end

            end

            assert_find_files_matches(root_dir,pat,is_recursive,expected);
        end
    end

function assert_find_files_matches(root_dir,pat,is_recursive,expected)
    result=moxunit_util_find_files(root_dir,pat,is_recursive);

    n_expected=numel(expected);

    full_expected=cell(n_expected,1);
    for k=1:n_expected
        full_expected{k}=fullfile(root_dir,expected{k}{:});
    end

    assertEqual(sort(full_expected(:)),sort(result(:)));


function [full_paths,is_dir]=make_files(root_dir,to_add,ext)
    n_paths=numel(to_add);

    full_paths=cell(1+n_paths,1);
    is_dir=false(1+n_paths,1);


    counter=0;
    if ~moxunit_util_isfolder(root_dir)
        counter=counter+1;
        mkdir(root_dir);
        full_paths{counter}=root_dir;
        is_dir(counter)=true;
    end


    ext_pat=[regexptranslate('escape',ext) '$'];

    for k=1:n_paths
        nm=fullfile(to_add{k}{:});

        is_file=~isempty(regexp(nm,ext_pat,'once'));

        p=fullfile(root_dir,nm);

        if is_file
            write_random_file(p);
        else
            mkdir(p);
        end

        counter=counter+1;
        full_paths{counter}=p;
        is_dir(counter)=~is_file;
    end

function write_random_file(p)
    fid=fopen(p,'w');
    cleaner=onCleanup(@()fclose(fid));
    fprintf(fid,'%s',randstr());


function remove_paths(full_paths)
    for k=numel(full_paths):-1:1
        p=full_paths{k};

        if ~ischar(p)
            % not a path, ignore
            continue;
        end

        if moxunit_util_isfolder(p)
            if(numel(dir(p))>2)
                error('Directory %s not empty',p);
            end

            rmdir(p);
        else
            delete(p);
        end
    end



