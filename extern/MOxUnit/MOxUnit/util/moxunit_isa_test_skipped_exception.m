function tf=moxunit_isa_test_skipped_exception(exception)
% return true if the input is a MOxUnit test-skipped exception
%
% tf=moxunit_isa_test_skipped_exception(exception)
%
% Input:
%   exception           an exception struct or object with field
%                       .identifier
%
% Output:
%   tf                  true if exception.identifer is equal to that
%                       thrown by moxunit_throw_test_skipped_exception
%
% Notes:
%   - if this exception is thrown when running a test through
%     a MOxUnitTestReport instance, this instance will record the test as
%     skipped, not as a failure
%
% NNO Jan 2014

    persistent cached_test_skipped_identifier

    if isempty(cached_test_skipped_identifier)
        try
            moxunit_throw_test_skipped_exception('error');
            assert(false,'should never get here');
        catch
            last_caught_error=lasterror();
        end

        cached_test_skipped_identifier=last_caught_error(1).identifier;
    end

    tf=strcmp(exception.identifier,...
                cached_test_skipped_identifier);



