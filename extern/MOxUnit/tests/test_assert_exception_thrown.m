function test_suite=test_assert_exception_thrown()
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

% Test cases where exceptions are thrown and that is OK
function test_assert_exception_thrown_passes
    assertExceptionThrown(@()error('Throw w/o ID'));
    assertExceptionThrown(@()error('moxunit:error','msg'),...
        'moxunit:error');
    assertExceptionThrown(@()error('moxunit:error','msg'),...
        'moxunit:error','message');
    assertExceptionThrown(@()error('moxunit:error','msg'),...
        'moxunit:error');
    assertExceptionThrown(@()error('moxunit:error','msg'),...
        '*','message');                                    % Same as above
    assertExceptionThrown(@()error('Throw w/o ID'),...
        '*','message');                                    % Same as above
    assertExceptionThrown(@()error('moxunit:error','msg'),...
        '*');                                              % Any error OK
    assertExceptionThrown(@()error('Throw w/o ID'),...
        '*');                                              % Same as above

    % Explicitly assert that an error is thrown without an ID
    assertExceptionThrown(@()error('Throw w/o ID'),...
        '');

    % Allow id to be a cellstr
    assertExceptionThrown(@()error('moxunit:error','msg'),...
        {'moxunit:error'},'message');
    assertExceptionThrown(@()error('Throw w/o ID'),...
        '*','message');
    assertExceptionThrown(@()error('moxunit:error','msg'),...
        {'moxunit:foo','moxunit:error','moxunit:foo'});
    assertExceptionThrown(@()error('Throw w/o ID'),...
        {'','moxunit:baz'});

function test_assert_exception_thrown_illegal_arguments
    args_cell={ ...
                {@()error('foo'),'message'},...
                {@()error('foo'),'not:id:_entifier','message'},...
                {@()error('foo'),struct},...
                {@()error('foo'),9},...
                {@()error('foo'),'error:id',9},...
                {@()error('foo'),'error:id',struct},...
                {@()error('foo'),'error_id','message'},...
                {'foo'},...
              };

    for k=1:numel(args_cell)
        args=args_cell{k};
        try
            assertExceptionThrown(args{:})

            error_exception_not_thrown('moxunit:illegalParameter');
        catch
            [unused,error_id]=lasterr();
            error_if_wrong_id_thrown('moxunit:illegalParameter',error_id);
        end
    end


% Test cases where func throws exceptions and we need to throw as well
function test_assert_exception_thrown_wrong_exception
    % Verify that when func throws but the wrong exception comes out, we respond
    % with the correct exception (assertExceptionThrown:wrongException)
    exception_id_cell = {'moxunit:failed',...
                         '',...
                         {'','moxunit:failed'},...
                        };

    for k=1:numel(exception_id_cell)
        exception_id=exception_id_cell{k};

        try
            assertExceptionThrown(@()error('moxunit:error','msg'),...
                exception_id,'msg');
            error_exception_not_thrown('moxunit:wrongExceptionRaised');
        catch
            [unused,error_id]=lasterr();
            error_if_wrong_id_thrown('moxunit:wrongExceptionRaised',...
                                                            error_id);
        end
    end

% Test cases where func does not throw but was expected to do so
function test_assert_exception_thrown_exceptions_not_thrown

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
            assertExceptionThrown(args{:});
            error_exception_not_thrown('moxunit:exceptionNotRaised');
        catch
            [unused,error_id]=lasterr();
            error_if_wrong_id_thrown('moxunit:exceptionNotRaised',...
                                        error_id);
        end
   end


function error_exception_not_thrown(error_id)
    error('moxunit:exceptionNotRaised', 'Exception ''%s'' not thrown', error_id);

function error_if_wrong_id_thrown(expected_error_id, thrown_error_id)
    if ~strcmp(thrown_error_id, expected_error_id)
        error('moxunit:wrongExceptionRaised',...
              'Exception raised with id ''%s'' expected id ''%s''',...
              thrown_error_id,expected_error_id);
    end


function do_nothing
    % do nothing