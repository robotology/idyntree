function label_counts=getTestOutputStatistics(obj)
% get number of failures, skipped and errored tests
%
% label_counts=getTestOutputStatistics(obj)
%
% Input:
%   obj             MOxUnitTestReport instance containg tests that have
%                   been run.
%
% Output:
%   label_counts    struct that may contain the fields 'failure', 'error',
%                   'skipped', with the value the number of times a text
%                   failed, raised an error or was skipped. If a field is
%                   absent, the number of times is zero.
%                   A success is not part of label_counts

    label_verbosity=2;
    label_counts=struct();

    for i=1:countTestOutcomes(obj)
        test_outcome=getTestOutcome(obj,i);

        if isSuccess(test_outcome)
            % no need to report statistics for successes
            continue;
        end

        test_label=getOutcomeStr(test_outcome,label_verbosity);

        if ~isfield(label_counts,test_label);
            label_counts.(test_label)=0;
        end

        label_counts.(test_label)=label_counts.(test_label)+1;
    end