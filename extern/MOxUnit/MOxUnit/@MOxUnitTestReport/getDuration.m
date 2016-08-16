function duration=getDuration(obj)
% get the duration of the test
%
% duration=getDuration(obj)
%
% Input:
%   obj                 MOxUnitTestReport instance
%
% Output:
%   duration            total time in seconds that it took to run all tests
%                       in the obj instance

    get_outcome_duration=@(i)getDuration(getTestOutcome(obj,i));

    all_durations=arrayfun(get_outcome_duration,...
                            1:countTestOutcomes(obj));

    duration=sum(all_durations);
