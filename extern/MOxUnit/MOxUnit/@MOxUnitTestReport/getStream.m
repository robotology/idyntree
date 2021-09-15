function stream=getStream(obj)
% get the output stream of the report
%
% stream=getStream(obj)
%
% Input:
%   obj                 MOxUnitTestReport instance
%
% Output:
%   stream              File identifier to which output is written.
%                       stream=1 means standard output

    stream=obj.stream;
