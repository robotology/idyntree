function result_str=moxunit_util_remove_matlab_anchor_tag(str)
% remove Matlab anchor tag that is inserted in error message
%
% result_str=moxunit_util_escape_xml(input_str)
%
% Input:
%   input_str           Input string with possible occurence of anchor tags
%                       from error message
%
%
% Output:
%   result_str          Output string with anchor tags removed.
%                       Any occurence of '<a foo>bar</a>' in input_str
%                       is replaced by 'bar' in output_str; but any
%                       content between double quote characters is ignored.
%
% Notes:
%   - this function is used to remove the anchor

pos=0;

anchor_start_open='<a';
anchor_start_close='>';

anchor_close='</a>';
ignore_inside='"';

result_str_cell=cell(0);
while true
    anchor_pos=find_str_pos(pos, str, anchor_start_open);
    if isempty(anchor_pos)
        % no more anchors, we are done
        result_str_cell{end+1}=str((pos+1):end);
        break;
    else
        result_str_cell{end+1}=str((pos+1):anchor_pos);
        pos=anchor_pos+numel(anchor_start_open);
    end

    while pos<numel(str)
        cur_char=str(pos);
        if strcmp(cur_char,ignore_inside)
            % find closing '"'
            pos=find_str_pos_after(pos, str, ignore_inside);
        elseif strcmp(cur_char,anchor_start_close)
            content_pos=find_str_pos(pos, str, anchor_close);
            result_str_cell{end+1}=str((pos+1):content_pos);

            % closing tag
            pos=content_pos+numel(anchor_close);
            break;
        else
            % next character
            pos=pos+1;
        end
    end
end


result_str=moxunit_util_strjoin(result_str_cell,'');

function i=find_str_pos(pos, str, needle)
% find the position before the first occurence of needle in str,
% starting at the first position after pos
    i=pos+regexp(str((pos+1):end),regexptranslate('escape',needle),'once')-1;

function i=find_str_pos_after(pos, str, needle)
% find the position immediately after the first occurence of needle in str,
% starting at position pos
    i=numel(needle)+1+find_str_pos(pos, str, needle);
