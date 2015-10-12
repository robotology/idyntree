tol = 1e-9;

dynComp = iDynTree.DynamicsComputations();

ok = dynComp.loadRobotModelFromFile('./model.urdf');

dynComp.getFloatingBase();

% set state
dofs = dynComp.getNrOfDegreesOfFreedom();
q = iDynTree.VectorDynSize(dofs);
dq = iDynTree.VectorDynSize(dofs);
ddq = iDynTree.VectorDynSize(dofs);

q.fromMatlab(rand(dofs,1));
dq.fromMatlab(rand(dofs,1));
ddq.fromMatlab(rand(dofs,1));

% set gravity
grav = iDynTree.SpatialAcc();
grav.setVal(2,-9.81);

dynComp.setRobotState(q,dq,ddq,grav);

torques = iDynTree.VectorDynSize(dofs);
baseReactionForce = iDynTree.Wrench();

% compute id with inverse dynamics
dynComp.inverseDynamics(torques,baseReactionForce);

% compute id with regressors
nrOfParams = 6*dynComp.getNrOfLinks();
regr = iDynTree.MatrixDynSize(6+dofs,nrOfParams);
params = iDynTree.VectorDynSize(nrOfParams);
dynComp.getDynamicsRegressor(regr);
dynComp.getModelDynamicsParameters(params);

generalizedForces = regr.toMatlab()*params.toMatlab();

disp('Result of inverse dynamics:');
torques.toMatlab()
baseReactionForce.toMatlab()

% check consistency
iDynTreeAssertEqual([baseReactionForce.toMatlab();torques.toMatlab()],generalizedForces,'Error in testDynComp');

disp('Test of DynamicsComputations completed successfully.');