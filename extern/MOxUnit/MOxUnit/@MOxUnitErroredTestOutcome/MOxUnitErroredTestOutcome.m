function obj=MOxUnitErroredTestOutcome(test_,duration,error_)
% Errored test outcome
%
% Inputs:
%   test_               MOxUnitTestCase of the test that errored
%   duration            time in seconds it took to run the test
%   error_              struct represting the contents of lasterror() after
%                       the test failed
%
% Returns:
%   obj                 MOxUnitErroredTestOutcome instance, which is a
%                       subclass of MoxUnitTestOutcome
%
% See also: lasterror, MoxUnitTestOutcome

    s=struct();
    s.error=error_;
    obj=class(s,'MOxUnitErroredTestOutcome',...
                MOxUnitTestOutcome(test_,duration));