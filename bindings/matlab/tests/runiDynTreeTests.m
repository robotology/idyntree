% This tests are designed to be executed from the build directory
% The .m files are copied in the build directory, and are executed there
addpath(genpath('../'))

% we need to use custom functions because the Matlab Unit Testing
% framework was introduced in Matlab 2013a, and we need to support
% also Matlab 2012x
try
    testSumPos
    testMomentumInvariance
    testTransforms
catch ME
    warning('iDynTree matlab tests failed. Exiting matlab.');
    warning(ME.identifier,ME.message);
    exit(1)
end


% if we arrive at this point all test went well
disp('iDynTree Test: all test completed successfully!')
exit(0)
