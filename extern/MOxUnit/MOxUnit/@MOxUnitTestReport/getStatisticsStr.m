function s=getStatisticsStr(obj, format)
    format2printer=struct();
    format2printer.text=@obj2stat_str_text;
    format2printer.xml=@obj2stat_str_xml;

    f=format2printer.(format);
    s=f(obj);

function s=obj2stat_str_xml(obj)
    % do not return any stats
    s='';

function status=obj2stat_str_text(obj)
    label_verbosity=2;
    label2count=struct();

    for i=1:countTestOutcomes(obj)
        test_outcome=getTestOutcome(obj,i);

        if isSuccess(test_outcome)
            % no need to report statistics for successes
            continue;
        end

        test_label=getOutcomeStr(test_outcome,label_verbosity);

        if ~isfield(label2count,test_label);
            label2count.(test_label)=0;
        end

        label2count.(test_label)=label2count.(test_label)+1;
    end

    if wasSuccessful(obj)
        status='OK';
    else
        status='FAILED';
    end

    labels=fieldnames(label2count);
    n_labels=numel(labels);

    if n_labels>0
        parts=cell(1,n_labels);
        for k=1:n_labels
            label=labels{k};
            parts{k}=sprintf('%s=%d',label,label2count.(label));
        end

        status=sprintf('%s (%s)',status,moxunit_util_strjoin(parts,', '));
    end
