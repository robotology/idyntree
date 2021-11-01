function test_suite=test_moxunit_util_stack2str
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_moxunit_util_stack2str_basics

    prefixes={[],'',randstr(),'% %d %s %%'};
    for k=1:numel(prefixes)
        n_elem=ceil(rand()*10+10);

        values=cell(3,n_elem); % file, name, line
        for j=1:n_elem
            values{1,j}=randstr();
            values{2,j}=randstr();
            values{3,j}=ceil(rand()*100);
        end

        keys={'file','name','line'};
        stack_arg_cell=arrayfun(@(i) {keys{i};values(i,:)},1:numel(keys),...
                                'UniformOutput',false);
        stack_arg=cat(1,stack_arg_cell{:});
        stack=struct(stack_arg{:});

        prefix=prefixes{k};
        if isequal(prefix,[])
            use_prefix='';
            prefix_arg={};
        else
            prefix_arg={prefix};
            use_prefix=prefix;
        end

        result=moxunit_util_stack2str(stack,prefix_arg{:});
        lines=regexp(result,sprintf('\n'),'split');
        assertEqual(numel(lines),n_elem);

        % compare other lines
        show_stack_elem=@(i)sprintf('%s%s:%d (%s)', ...
                            use_prefix, stack(i).name, ...
                            stack(i).line, stack(i).file);
        expected_lines=arrayfun(show_stack_elem,1:n_elem,...
                            'UniformOutput',false);
        assertEqual(lines,expected_lines);

    end



function s=randstr()
    s=char(rand(1,10)*24+65);

function test_moxunit_util_stack2str_exceptions
    aet=@(varargin)assertExceptionThrown(@()...
                            moxunit_util_stack2str(varargin{:}),'');

    % first argument must be struct
    aet('foo')
    aet([]);
    aet({})

    % second argument must be a string
    aet(struct(),{});
    aet(struct(),[]);
    aet(struct(),struct());

    % bad input, missing fields
    s_missing_field=struct();
    s_missing_field.file='foo';
    aet(s_missing_field);

    % should be ok
    s_octave=struct();
    s_octave.file='foo';
    s_octave.name='bar';
    s_octave.line=23;
    s_octave.column=2;
    s_octave.scope=3;
    s_octave.context=[];
    moxunit_util_stack2str(s_octave);



function str=stack2str(stack)
    n_stack=numel(stack);
    lines=cell(1,n_stack);
    for k=1:n_stack
        s=stack(k);
        lines{k}=sprintf('  %s:%d (%s)', ...
                        s.name, s.line, s.file);
    end
    str=moxunit_util_strjoin(lines,'\n');