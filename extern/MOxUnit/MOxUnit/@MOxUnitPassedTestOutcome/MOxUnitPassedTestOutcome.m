function obj=MOxUnitPassedTestOutcome(test_,duration)
% Passed test outcome
%
% Inputs:
%   test_               MOxUnitTestCase of the test that failed
%   duration            time in seconds it took to run the test
%   error_              struct represting the contents of lasterror() after
%                       the test failed
%
% Returns:
%   obj                 MOxUnitPassedTestOutcome instance, which is a
%                       subclass of MoxUnitTestOutcome
%
% See also: lasterror, MoxUnitTestOutcome

    s=struct();
    obj=class(s,'MOxUnitPassedTestOutcome',...
                MOxUnitTestOutcome(test_,duration));