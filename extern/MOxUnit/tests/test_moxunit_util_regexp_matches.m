function test_suite=test_moxunit_util_regexp_matches()
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function c=randchar(length)
    c=char(ceil(rand(1,length)*26+64));

function test_moxunit_util_regexp_matches_basics
    for needle_length=1:3

        %  make string to match
        needle=randchar(needle_length);
        assert_regexp_matches(needle,needle)

        for pat_length=1:4
            % make pattern that matches or does not match needle
            pat=randchar(pat_length);
            assert_regexp_matches([needle pat],needle)
            assert_regexp_matches([pat needle],needle)

            if needle_length==pat_length
                matcher=@assert_regexp_matches;
            else
                matcher=@assert_regexp_not_matches;
            end
            matcher(needle,sprintf('^%s$',...
                                        repmat('.',1,pat_length)))


            while true
                different_haystack=randchar(pat_length);
                match=bsxfun(@eq,different_haystack,needle');
                if all(~match(:))
                    break;
                end
            end

            assert_regexp_not_matches(needle,different_haystack)
        end
    end

function test_moxunit_util_regexp_matches_exceptions()
    aet=@(varargin)assertExceptionThrown(@()...
                        moxunit_util_regexp_matches(varargin{:}),'');
    bad_inputs={{[],''},... % first argument not a string
                 {'',[]},... % second argument not a string
                 };
    for k=1:numel(bad_inputs)
        args=bad_inputs{k};
        aet(args{:});
    end


function assert_regexp_matches(needle, pat)
    assertTrue(moxunit_util_regexp_matches(needle,pat));

function assert_regexp_not_matches(needle, pat)
    assertFalse(moxunit_util_regexp_matches(needle,pat));


