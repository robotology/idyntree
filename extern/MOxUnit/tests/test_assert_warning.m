function test_suite=test_assert_warning()
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

% Test cases where warnings are thrown and that is OK
function test_assert_warning_passes
    assertWarning(@()warning('Throw w/o ID'));
    assertWarning(@()warning('moxunit:warning','msg'),...
        'moxunit:warning');
    assertWarning(@()warning('moxunit:warning','msg'),...
        'moxunit:warning','message');
    assertWarning(@()warning('moxunit:warning','msg'),...
        'moxunit:warning');
    assertWarning(@()warning('moxunit:warning','msg'),...
        '*','message');                                    % Same as above
    assertWarning(@()warning('Throw w/o ID'),...
        '*','message');                                    % Same as above
    assertWarning(@()warning('moxunit:warning','msg'),...
        '*');                                              % Any warning OK
    assertWarning(@()warning('Throw w/o ID'),...
        '*');                                              % Same as above

    % Allow id to be a cellstr
    assertWarning(@()warning('moxunit:warning','msg'),...
        {'moxunit:warning'},'message');
    assertWarning(@()warning('Throw w/o ID'),...
        '*','message');
    assertWarning(@()warning('moxunit:warning','msg'),...
        {'moxunit:foo','moxunit:warning','moxunit:foo'});
    assertWarning(@()warning('Throw w/o ID'),...
        {'','moxunit:baz'});

function test_assert_warning_illegal_arguments
    args_cell={ ...
                {@()warning('foo'),'message'},...
                {@()warning('foo'),'not:id:_entifier','message'},...
                {@()warning('foo'),struct},...
                {@()warning('foo'),9},...
                {@()warning('foo'),'warning:id',9},...
                {@()warning('foo'),'warning:id',struct},...
                {@()warning('foo'),'warning_id','message'},...
                {'foo'},...
              };

    for k=1:numel(args_cell)
        args=args_cell{k};
        try
            assertWarning(args{:})

            error_warning_not_thrown('moxunit:illegalParameter');
        catch
            [unused,warning_id]=lasterr();
            error_if_wrong_id_thrown('moxunit:illegalParameter',warning_id);
        end
    end


% Test cases where func throws warnings and we need to throw as well
function test_assert_warning_wrong_warning
    % Verify that when func throws but the wrong warning comes out, we respond
    % with the correct warning (assertWarning:wrongWarning)
    warning_id_cell = {'moxunit:failed',...
                         '',...
                         {'','moxunit:failed'},...
                        };

    for k=1:numel(warning_id_cell)
        warning_id=warning_id_cell{k};

        try
            assertWarning(@()warning('moxunit:warning','msg'),...
                warning_id,'msg');
            error_warning_not_thrown('moxunit:wrongWarningRaised');
        catch
            [unused,warning_id]=lasterr();
            error_if_wrong_id_thrown('moxunit:wrongWarningRaised',...
                                                            warning_id);
        end
    end

% Test cases where func does not throw but was expected to do so
function test_assert_warning_warnings_not_thrown

    % For all combination of optional arguments we expect the same
    % behavior. We will loop, testing all combinations here
    args_cell = {...
                    {@do_nothing},...                   % No arguments
                    {@do_nothing,'moxunit:failed'},...  % Identifier only
                    {@do_nothing,'*','message'},...     % Wildcard
                    {@do_nothing,'a:b','msg'},...       % All arguments
                    {@do_nothing,{'a:b','c:d'},'msg'}   % Cell str
                 };

   for k=1:numel(args_cell)
        args=args_cell{k};

        % Run the test
        try
            assertWarning(args{:});
            error_warning_not_thrown('moxunit:warningNotRaised');
        catch
            [unused,warning_id]=lasterr();
            error_if_wrong_id_thrown('moxunit:warningNotRaised',...
                                        warning_id);
        end
   end


function error_warning_not_thrown(warning_id)
    error('moxunit:warningNotRaised', 'Warning ''%s'' not thrown', warning_id);

function error_if_wrong_id_thrown(expected_warning_id, thrown_warning_id)
    if ~strcmp(thrown_warning_id, expected_warning_id)
        error('moxunit:wrongWarningRaised',...
              'Warning raised with id ''%s'' expected id ''%s''',...
              thrown_warning_id, expected_warning_id);
    end


function do_nothing
    % do nothing