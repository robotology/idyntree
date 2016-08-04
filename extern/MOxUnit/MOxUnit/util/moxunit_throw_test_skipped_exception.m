function moxunit_throw_test_skipped_exception(reason)
% throw an exception signalling a test-skipped for MOxUnit
% return true if the input is a MOxUnit test-skipped exception
%
% moxunit_throw_test_skipped_exception(reason)
%
% Input:
%   reason                  a string containing the reason for skipping the
%                           test
% Throws:
%   'moxUnit:testSkipped'   exception to signal that the test is skipped
%
% Notes:
%   - if this exception is thrown when running a test through
%     a MOxUnitTestReport instance, this instance will record the test as
%     skipped, not as a failure
%
% NNO Jan 2014

    error('moxUnit:testSkipped','%s',reason);
