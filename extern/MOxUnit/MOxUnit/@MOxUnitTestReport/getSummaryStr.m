function str=getSummaryStr(obj, format)
% get summary of tests that were run and their output
%
% str=getSummaryStr(obj, format)
%
% Inputs:
%   obj                 MOxUnitTestReport instance
%   format              Output format: 'xml' or 'text'
%
% Output:
%   str                 text or XML string representation of the results of
%                       the tests
%

    format2converter=struct();
    format2converter.text=@report2summary_text;
    format2converter.xml=@report2summary_xml;

    f=format2converter.(format);
    str=f(obj);

function str=report2summary_xml(obj)
% return summary in XML format
    header=sprintf('<testsuite name="%s" time="%.3f" %s>',...
                    moxunit_util_escape_xml(getName(obj)),...
                    getDuration(obj),...
                    statistics_xml(obj));

    summary_text_cell=get_outcome_summary_cell(obj,'xml');
    body=moxunit_util_strjoin(summary_text_cell,'\n');

    footer='</testsuite>';
    str=sprintf('%s\n%s\n%s',header,body,footer);


function str=report2summary_text(obj)
% return summary in text format
    verbosity=obj.verbosity;

    hor_line=repmat('-',1,50);

    body='';
    if verbosity>=2 || ~wasSuccessful(obj)
        summary_text_cell=get_outcome_summary_cell(obj,'text');
        non_empty_msk=~cellfun(@isempty,summary_text_cell);
        if any(non_empty_msk)
            summary_text_cell_nonempty=summary_text_cell(non_empty_msk);
            body_non_pass=moxunit_util_strjoin(...
                                summary_text_cell_nonempty,'\n\n');
            body=sprintf('\n%s\n\n%s\n%s\n\n',hor_line,body_non_pass);
        end
    end


    if verbosity>=1
        footer=sprintf('\n%s\n\n%s',hor_line,statistics_text(obj));
    else
        footer='';
    end

    str=sprintf('%s%s',body,footer);



function [c,n_tests]=get_outcome_summary_cell(obj, format)
% helper function to get the summary from each test
    n_tests=countTestOutcomes(obj);
    c=cellfun(@(x)getSummaryStr(getTestOutcome(obj,x),format),...
                    num2cell(1:n_tests),...
                    'UniformOutput',false);

function str=statistics_xml(obj)
% return xml string indicating number of tests, and how many were failures,
% skips and errors
%
% Example outputs:
%       'tests="9"'
%       'tests="9" failures="2"'

    total_tests=sprintf('tests="%d"',countTestOutcomes(obj));

    labels_singular_plural={'failure','failures';...
                            'skipped','skipped';...
                            'error','errors'};
    n_labels=size(labels_singular_plural,1);

    label2count=getTestOutputStatistics(obj);
    non_success_tests_cell=cell(n_labels,1);
    for k=1:n_labels
        singular=labels_singular_plural{k,1};
        plural=labels_singular_plural{k,2};

        if isfield(label2count,singular)
            count=label2count.(singular);
        else
            count=0;
        end

        non_success_tests_cell{k}=sprintf('%s="%d"',plural,count);
    end

    str=sprintf('%s %s',total_tests,...
                    moxunit_util_strjoin(non_success_tests_cell,' '));




function str=statistics_text(obj)
% return text string indicating whether all tests passed or not, and
% number of failures, skips and errors.
%
% Example outputs:
%       'OK'
%       'FAILED (skipped=2)'

    if wasSuccessful(obj)
        str='OK';
    else
        str='FAILED';
    end

    label_counts=getTestOutputStatistics(obj);
    labels=fieldnames(label_counts);

    n_labels=numel(labels);
    if n_labels>0
        cell_stats=cellfun(@(x) sprintf('%s=%d',x, label_counts.(x)),...
                                    fieldnames(label_counts),...
                                    'UniformOutput',false);
        str=sprintf('%s (%s)',str,moxunit_util_strjoin(cell_stats,', '));
    end






