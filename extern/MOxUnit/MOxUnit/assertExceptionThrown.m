function assertExceptionThrown(func, expected_id, message)
% Assert that an exception is thrown
%
% assertExceptionThrown(func, [expected_id,[message]])
%
% Inputs:
%   func            Function handle that is expected to throw, with the
%                   prototype
%                       [varargout{:}] = func()
%   expected_id     Identifier of the expected exception (optional). If
%                   not provided, then func can raise any exception.
%                   To allow for any exception to be raised, use '*'
%   message         Custom message to be included when func fails to throw
%
% Throws:
%   'moxunit:exceptionNotRaised'    func() does not raise an exception but
%                                   was expected to do so.
%   'moxunit:wrongExceptionRaised'  func() does raise an exception but with
%                                   an identifier different from
%                                   expected_id
%
%
% Examples:
%   % Assert that sin will throw an exception when its argument is a struct
%   >> assertExceptionThrown( @()sin( struct([]) ) )
%
%   % Assert that sin throws AND that the ID is 'MATLAB:UndefinedFunction'
%   >> assertExceptionThrown( @()sin( struct([]) ), ...
%                                   'MATLAB:UndefinedFunction')
%
% Notes:
% - This function allows one to test for exceptions being thrown, and
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

    try
        func();

        [id,whats_wrong]=exception_not_raised(expected_id);
    catch
        % (Avoiding '~' for Octave compatibility)
        [unused,found_id] = lasterr();

        if exception_id_matches(expected_id,found_id)
            % the expected exception was raised, we're done
            return;
        end

        [id,whats_wrong]=wrong_exception_raised(found_id, expected_id);
    end

    full_message=moxunit_util_input2str(message,whats_wrong);

    if moxunit_util_platform_is_octave()
        error(id,full_message);
    else
        throwAsCaller(MException(id, full_message));
    end


function [id,whats_wrong]=exception_not_raised(expected_id)
    id = 'moxunit:exceptionNotRaised';
    whats_wrong='No exception was raised';

    if ~strcmp(expected_id,'*')
        % add suffix with expected id
        whats_wrong=sprintf('%s, expected exception ''%s''',...
                                        whats_wrong,expected_id);
    end


function [id,whatswrong]=wrong_exception_raised(found_id, expected_id)
    id='moxunit:wrongExceptionRaised';
    whatswrong = sprintf(...
        'exception ''%s'' was raised, expected ''%s''',...
        found_id, expected_id);


function verify_inputs(varargin)
    expected_types={'function_handle','char','char'};
    for k=1:numel(expected_types)
        expected_type=expected_types{k};
        if ~isa(varargin{k},expected_type)
            error('moxunit:illegalParameter',...
                        ['Argument at position %d must be '...
                        'of type ''%s'', found ''%s'''],...
                        k,expected_type,class(varargin{k}));
        end
    end

    expected_id=varargin{2};
    if ~(strcmp(expected_id,'') || ...
                strcmp(expected_id,'*') || ...
                moxunit_util_is_message_identifier(expected_id))
        error('moxunit:illegalParameter',...
                        ['Argument at position 2, ''expected_id'', '...
                        'must be empty, an asterisk (''*''), or '...
                        'a valid message identifier, found ''%s'''],...
                        expected_id);
    end




function tf=is_wildcard_id(id)
    tf=strcmp(id,'*');

function tf=exception_id_matches(expected_id,found_id)
    tf=is_wildcard_id(expected_id) || isequal(expected_id,found_id);

