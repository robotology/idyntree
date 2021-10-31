function writeXML(obj,fn)
% write test report in JUnit XML format
%
% writeXML(obj,fn)
%
% Inputs:
%   obj                 MOxUnitTestReport instance
%   fn                  filename to which results are written
%

    fid=fopen(fn,'w');
    file_closer=onCleanup(@()fclose(fid));

    xml_preamble='<?xml version="1.0" encoding="utf-8"?>';
    xml_header='<testsuites>';
    xml_body=getSummaryStr(obj,'xml');
    xml_footer='</testsuites>';

    fprintf(fid,'%s\n%s\n%s\n%s',...
                xml_preamble,...
                xml_header,...
                xml_body,...
                xml_footer);