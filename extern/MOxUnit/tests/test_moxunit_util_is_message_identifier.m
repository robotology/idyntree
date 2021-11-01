function test_suite=test_moxunit_util_is_message_identifier
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_moxunit_util_is_message_identifier_non_char
    aet=@(varargin)assertExceptionThrown(@()...
                    moxunit_util_is_message_identifier(varargin{:}),'');

    aet(struct());                      % struct
    aet({'id:id2'});                    % cell
    aet(2);                             % numeric
    aet(false);                         % boolean
    aet(true);                          % boolean
    aet([]);                          % boolean

function test_moxunit_util_is_message_identifier_true
    true_strs={'a:b',...
                'a___:b___',...
                'abc:def:ghi:jkl:mno',...
                'a_b:c_123',...
                };

    for k=1:numel(true_strs)
        s=true_strs{k};
        assertTrue(moxunit_util_is_message_identifier(s),s);
    end


function test_test_moxunit_util_is_message_identifier_false
    false_strs={'',...                  % empty string
                'a',...                 % missing colon
                'ab',...                % missing colon
                'ab: cd',...            % contains space
                sprintf('ab:c\td'),...  % contains tabs
                '_a:b',...              % starts with underscore
                'a:_b',...              % starts with underscore
                '1ab:def',...           % has a number
                ':abc:def:',...         % starts with colon
                'abc:def:',...          % ends with colon
                'a-b',...               % no colon
                };


    for k=1:numel(false_strs)
        s=false_strs{k};

        assertFalse(moxunit_util_is_message_identifier(s),s);
    end

function test_test_moxunit_util_is_message_identifier_octave
    % in octave, the '-' is allowed to be in message identifers;
    % in Matlab this is not allowed.
    % For compatibility we allow the '-' to be allowed in message
    % identifiers
    true_strs={'a-b:b:cdef-ghij',...
                'a-b:c',...
                'a:b-c',...
                };

    for k=1:numel(true_strs)
        s=true_strs{k};
        assertTrue(moxunit_util_is_message_identifier(s),s);
    end
