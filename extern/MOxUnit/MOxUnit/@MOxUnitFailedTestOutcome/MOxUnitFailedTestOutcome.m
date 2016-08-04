function obj=MOxUnitFailedTestOutcome(test_,duration,error_)
% Failed test outcome
%
% Inputs:
%   test_               MOxUnitTestCase of the test that failed
%   duration            time in seconds it took to run the test
%   error_              struct represting the contents of lasterror() after
%                       the test failed
%
% Returns:
%   obj                 MOxUnitFailedTestOutcome instance, which is a
%                       subclass of MoxUnitTestOutcome
%
% See also: lasterror, MoxUnitTestOutcome

    s=struct();
    s.error=error_;
    obj=class(s,'MOxUnitFailedTestOutcome',...
                MOxUnitTestOutcome(test_,duration));