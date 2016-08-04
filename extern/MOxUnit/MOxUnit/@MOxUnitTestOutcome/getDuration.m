function duration=getDuration(obj)
% get the duration of the test
%
% duration=getDuration(obj)
%
% Input:
%   obj                 MOxUnitTestOutcome instance
%
% Output:
%   duration            time in seconds that it took to run the test


    duration=obj.duration;