function str_esc=moxunit_util_escape_xml(str)
% escapes characters for XML representation
%
% str_esc=moxunit_util_escape_xml(str)
%
% Input:
%   str                 Input string to be escaped
%
% Output:
%   str_esc             String with the followed characters escaped:
%                       original    escaped
%                       ===================
%                           &       &amp;
%                           >       &gt;
%                           <       &lt;
%                           "       &quot;
%                           '       %apos;

% Handle the ampersand first to avoid that output from other symbols will
% be escaped. On purpose the ampersand is not part of the table below, to
% avoid errors introduced by accidental re-orderings of the table

str_esc=strrep(str,'&','&amp;');

table={'>','&gt;';...
       '<','&lt;';...
       '"','&quot;';...
       '''','&apos;'};

n=size(table,1);
for k=1:n
    str_esc=strrep(str_esc,table{k,1},table{k,2});
end