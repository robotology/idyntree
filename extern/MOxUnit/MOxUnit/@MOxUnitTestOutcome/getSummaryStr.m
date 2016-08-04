function str=getSummaryStr(obj, format)

    formatters=struct();
    formatters.text=@obj2text;
    formatters.xml=@obj2xml;

    f=formatters.(format);

    str=f(obj);


function str=obj2text(obj)
    content=getSummaryContent(obj);

    % set outcome to 'failure', 'error', 'skipped', or 'passed'
    outcome_verbosity=2;
    outcome=getOutcomeStr(obj, outcome_verbosity);

    if isstruct(content)
        % error or failure
        str=sprintf('%s: %s\n%s',...
                outcome,content.message,stack2str(content.stack));

    elseif ischar(content)
        % skipped
        str=sprintf('%s: %s', outcome, content);

    elseif isequal(content,[])
        % passed
        str='';

    else
        assert(false,'this should not happen');
    end


function str=obj2xml(obj)
    test_=getTest(obj);

    prefix=sprintf('<testcase classname="%s" name="%s" time="%.3f"',...
                moxunit_util_escape_xml(get_classname(test_)),...
                moxunit_util_escape_xml(getName(test_)),...
                getDuration(obj));

    if isSuccess(obj)
        infix='';
        suffix=' />';
    else
        content=getSummaryContent(obj);

        % outcome is 'failure', 'error', or 'skipped'
        outcome_verbosity=2;
        outcome=getOutcomeStr(obj, outcome_verbosity);

        if isNonFailure(obj)
            % skipped
            % note: showing the reason is not supported
            infix=sprintf('>\n  <%s />',outcome);
        else
            % XXX currently the entire error message is shown, together
            % with the stack trace. It may be more convenient to show a
            % summary of this information
            raw_message=content.message;
            message=moxunit_util_remove_matlab_anchor_tag(raw_message);
            message=moxunit_util_escape_xml(message);
            stack_trace=moxunit_util_escape_xml(stack2str(content.stack));
            infix=sprintf('>\n  <%s message="%s">%s</%s>',...
                        outcome,message,stack_trace,outcome);
        end

        suffix=sprintf('\n</testcase>');
    end

    str=sprintf('%s%s%s',prefix,infix,suffix);

function str=stack2str(stack)
    n_stack=numel(stack);
    lines=cell(1,n_stack);
    for k=1:n_stack
        s=stack(k);
        lines{k}=sprintf('  %s:%d (%s)', ...
                        s.name, s.line, s.file);
    end
    str=moxunit_util_strjoin(lines,'\n');

function s=get_classname(test_)
    location=getLocation(test_);
    [pth,file_name]=fileparts(location);

    s=sprintf('%s',file_name);


