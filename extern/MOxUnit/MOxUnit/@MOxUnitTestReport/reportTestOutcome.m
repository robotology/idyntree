function obj=reportTestOutcome(obj, test_outcome)
% reports the test outcome to the stream of the current object
%
% obj=reportTestOutcome(obj, test_outcome)
%
% Inputs:
%   obj             MOxUnitTestReport instance
%   test_outcome    MOxUnitTestReport instance with the outcome to report.
%                   This function will print the outcome to the stream used
%                   when obj was instantiated. As a side effect,
%                   test_outcome is added to the report
%
% Output:
%   obj             MOxUnitTestReport instance with the test_outcome added

    obj=addTestOutcome(obj,test_outcome);

    verbosity=obj.verbosity;

    outcome_str=getProgressStr(test_outcome, verbosity);
    stream=obj.stream;

    postfix=''; % the default

    max_line_length=60;
    if mod(countTestOutcomes(obj),max_line_length)==0 && verbosity==1
        % when showing single characters, print a newline every sixty
        % characters
        postfix=sprintf('\n');
    end

    fprintf(stream,'%s%s',outcome_str,postfix);




