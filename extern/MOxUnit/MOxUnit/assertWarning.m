function assertWarning(func, expected_id, message)
% Assert that an warning is thrown
%
% assertWarning(func, [expected_id,[message]])
%
% Inputs:
%   func            Function handle that is expected to throw, with the
%                   prototype
%                       [varargout{:}] = func()
%   expected_id     Identifier of the expected warning (optional). If
%                   not provided, then func can raise any warning.
%                   To allow for any warning to be raised, use '*'.
%                   Multiple ids can be provided in a cell string; the
%                   assertion means that at least one id matches the one
%                   that is thrown.
%   message         Custom message to be included when func fails to throw
%
% Throws:
%   'moxunit:warningNotRaised'    func() does not raise an warning but
%                                   was expected to do so.
%   'moxunit:wrongWarningRaised'  func() does raise an warning but with
%                                   an identifier different from
%                                   expected_id
%   'moxunit:illegalParameter'      the input arguments are of the wrong
%                                   type or content
%
%
% Examples:
%   % Assert that solving with a singular matrix give  a warning
%   assertWarning( @() zeros(2) \ ones(2,1) );
%   %|| % ok
%
%   % Assert that solving with a singular matrix give a warning
%   % AND that the ID is either 'Octave:singular-matrix' or
%   % 'MATLAB:singularMatrix'
%   allowed_ids={'Octave:singular-matrix','MATLAB:singularMatrix'};
%   assertWarning( @() zeros(2) \ ones(2,1), allowed_ids);
%
%   % No warning raised
%   assertWarning(@()disp('hello world'));
%   %|| error('No warning was raised');
%
% Notes:
% - This function allows one to test for warnings being thrown, and
%   optionally, pass a custome message in response to a failure.
% - It is assumed that all input variables are of the correct type, valid
%   (if applicable), and given in the correct order.
%
% See also: moxunit_util_is_message_identifier

    if nargin<3
        message='';
    end

    if nargin<2
        expected_id='*';
    end

    verify_inputs(func, expected_id, message);

    state = warning('query');
    try
        % In octave, silencing warnings does not allow us to
        % get them through lastwarn
        warning('error', 'all');
    catch
        % In matlab, setting all warnings to errors is not allowed
        warning('off', 'all');
        if ~is_wildcard_id(expected_id)
            if ~iscellstr(expected_id)
                expected_id = {expected_id};
            end
            for i=expected_id
                if ~isempty(i{:})
                    warning('error', i{:});
                end
            end
        end
    end

    lastwarn('', '');

    try
        func();

        [message,warning_id] = lastwarn();

        if ~isempty(message) || ~isempty(warning_id)
            if ~isempty(warning_id)
                error(warning_id, message);
            else
                error(message);
            end
        end

        [id,whats_wrong]=warning_not_raised(expected_id);
    catch
        % (Avoiding '~' for Octave compatibility)
        [unused,found_id] = lasterr();

        if warning_id_matches(expected_id,found_id)
            % the expected warning was raised, we're done
            for i=1:numel(state)
                warning(state(i).state, state(i).identifier);
            end
            return;
        end

        [id,whats_wrong]=wrong_warning_raised(found_id, expected_id);
    end

    for i=1:numel(state)
        warning(state(i).state, state(i).identifier);
    end

    full_message = moxunit_util_input2str(message, whats_wrong);

    if moxunit_util_platform_is_octave()
        error(id,'%s',full_message);
    else
        throwAsCaller(MException(id,'%s',full_message));
    end


function [id,whats_wrong]=warning_not_raised(expected_id)
    id = 'moxunit:warningNotRaised';
    whats_wrong='No warning was raised';

    if ~any(strcmp(expected_id,'*'))
        % add suffix with expected id
        whats_wrong=sprintf('%s, expected warning %s',...
                            whats_wrong,expected_id2str(expected_id));
    end


function [id,whatswrong]=wrong_warning_raised(found_id, expected_id)
    last_err=lasterror();
    stack=last_err.stack;

    stack_indent_length=8;
    stack_indent=repmat(' ',1,stack_indent_length);
    %stack_indent='    | '
    stack_trace_str=moxunit_util_stack2str(stack,stack_indent);

    id='moxunit:wrongWarningRaised';
    whatswrong = sprintf(...
        ['warning ''%s'' was raised, expected %s. '...
        'Stack trace:\n\n%s'],...
        found_id, expected_id2str(expected_id), stack_trace_str);

function expected_str=expected_id2str(expected_id)
    if iscellstr(expected_id) && numel(expected_id)>1
        expected_str=sprintf('one of ''%s''',...
                        moxunit_util_strjoin(expected_id,''', '''));
    else
        if iscellstr(expected_id)
            expected_id=expected_id{1};
        end
        expected_str=sprintf('''%s''',expected_id);
    end


function verify_inputs(varargin)
    expected_type_funcs={@(x)isa(x,'function_handle'),...
                        @(x)ischar(x) || iscellstr(x),...
                        @ischar,...
                        };
    for k=1:numel(expected_type_funcs)
        expected_type_func=expected_type_funcs{k};
        arg=varargin{k};
        if ~expected_type_func(arg)
            error('moxunit:illegalParameter',...
                        ['Argument at position %d of wrong type, '...
                        'found ''%s'''],...
                        k,class(varargin{k}));
        end
    end

    expected_id=varargin{2};
    if ~is_valid_error_id(expected_id)
        error('moxunit:illegalParameter',...
                        ['Argument at position 2, ''expected_id'', '...
                        'must be a cellstring or single string; each '...
                        'string must be empty, an asterisk (''*''), or '...
                        'a valid message identifier, found ''%s'''],...
                        expected_id);
    end

function tf=is_valid_error_id(expected_id)
    if iscellstr(expected_id)
        % recursive call
        tf=all(cellfun(@is_valid_error_id,expected_id));
        return
    end

    tf=strcmp(expected_id,'') ...
                || strcmp(expected_id,'*') ...
                || iscellstr(expected_id) ...
                || moxunit_util_is_message_identifier(expected_id);



function tf=is_wildcard_id(id)
    tf=any(strcmp(id,'*'));

function tf=warning_id_matches(expected_id,found_id)
    if iscellstr(expected_id)
        % recursive call
        tf=any(cellfun(@(x)warning_id_matches(x,found_id),expected_id));
        return;
    end

    tf=is_wildcard_id(expected_id) ...
            || isequal(expected_id,found_id);
