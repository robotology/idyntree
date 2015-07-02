addpath(genpath('/home/pegua/src/codyco-superbuild/build/install/mex'))

% we need to use custom functions because the Matlab Unit Testing
% framework was introduced in Matlab 2013a, and we need to support
% also Matlab 2012x
testSumPos
testMomentumInvariance

% if we arrive at this point all test went well
disp('iDynTree Test: all test completed successfully!')
exit(0)
