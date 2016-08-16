function disp(obj)
% Display test results
%
% disp(obj)
%
%   obj             MOxUnitTestReport instance.
%
% Notes:
%   - this function writes results to the file descriptor provided when
%     obj was instantiated, and with the provided verbosity level
%
% NNO 2015


    output_format='text';
    fprintf(obj.stream,'%s\n',getSummaryStr(obj,output_format));
