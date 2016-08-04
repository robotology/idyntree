function obj=MOxUnitSkippedTestOutcome(test_,duration,reason)
% Skipped test outcome
%
% Inputs:
%   test_               MOxUnitTestCase of the test that failed
%   duration            time in seconds it took to run the test
%   reason              string containing the reason why the test was
%                       skipped
%
% Returns:
%   obj                 MOxUnitSkippedTestOutcome instance, which is a
%                       subclass of MoxUnitTestOutcome
%
% See also: lasterror, MoxUnitTestOutcome

    s=struct();
    s.reason=reason;
    obj=class(s,'MOxUnitSkippedTestOutcome',...
                MOxUnitTestOutcome(test_,duration));