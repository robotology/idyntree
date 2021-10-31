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
        stack_prefix='  ';
        stack_str=moxunit_util_stack2str(content.stack, stack_prefix);
        str=sprintf('%s: %s\n%s',...
                outcome,content.message,stack_str);

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

            raw_stack_str=moxunit_util_stack2str(content.stack);
            stack_str=moxunit_util_escape_xml(raw_stack_str);

            infix=sprintf('>\n  <%s message="%s">%s</%s>',...
                        outcome,message,stack_str,outcome);
        end

        suffix=sprintf('\n</testcase>');
    end

    str=sprintf('%s%s%s',prefix,infix,suffix);



function s=get_classname(test_)
    location=getLocation(test_);
    [pth,file_name]=fileparts(location);

    s=sprintf('%s',file_name);


