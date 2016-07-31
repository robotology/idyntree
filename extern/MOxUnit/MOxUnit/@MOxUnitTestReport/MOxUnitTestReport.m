function obj=MOxUnitTestReport(verbosity,stream,name)
% Initialize empty MOxUnitTestReport instance
%
% obj=MOxUnitTestReport([verbosity[,stream]])
%
% Inputs:
%   verbosity       Integer indicating how verbose the output is when
%                   tests are run using this instance (default: 1).
%   stream          File descriptor into which output results are written
%                   (default: 1, corresponding to standard output in the
%                   Command Window).
%
% Returns:
%   obj             Empty MOxUnitTestReport instance, with no test errors,
%                   failures, skips, or successes stored.
%
% See also: addError, addFailure, addSkip, addSuccess, report
%
% NNO 2015

    if nargin<1
        verbosity=1;
    end

    if nargin<2
        stream=1;
    end

    class_name='MOxUnitTestReport';
    if nargin<3
        name=class_name;
    end

    s=struct();
    s.verbosity=verbosity;
    s.stream=stream;
    s.name=name;
    s.test_outcomes=cell(0);
    obj=class(s,class_name);

