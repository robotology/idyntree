%% Load the estimator

% Create estimator class
estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();

% Load model and sensors from the URDF file
estimator.loadModelAndSensorsFromFile('./iCubGenova02.urdf');

% Check if the model was correctly created by printing the model
estimator.model().toString()

%% Set kinematics information

% Set kinematics information: for this example, we will assume
% that the robot is balancing on the left foot. We can then
% compute the kinematics information necessary for the FT sensor
% measurements estimation using the knoledge of the gravity on a
% a frame fixed to the l_foot link (for convenience we use the l_sole
% frame). For more info on iCub frames check: http://wiki.icub.org/wiki/ICub_Model_naming_conventions
grav_idyn = iDynTree.Vector3();
grav = [0.0;0.0;-9.81];
grav_idyn.fromMatlab(grav);

% Get joint information. for the time being we can just assume that
% all the joints position, velocity and acceleration are zero.
% Warning!! iDynTree takes in input **radians** based units,
% while the iCub port stream **degrees** based units.
dofs = estimator.model().getNrOfDOFs();

qj = zeros(dofs,1);
dqj = zeros(dofs,1);
ddqj = zeros(dofs,1);

qj_idyn   = iDynTree.JointPosDoubleArray(dofs);
dqj_idyn  = iDynTree.JointDOFsDoubleArray(dofs);
ddqj_idyn = iDynTree.JointDOFsDoubleArray(dofs);

qj_idyn.fromMatlab(qj);
dqj_idyn.fromMatlab(dqj);
ddqj_idyn.fromMatlab(ddqj);

% Set the kinematics information in the estimator
l_sole_index = estimator.model().getFrameIndex('l_sole');
estimator.updateKinematicsFromFixedBase(qj_idyn,dqj_idyn,ddqj_idyn,l_sole_index,grav_idyn);

%% Specify unknown wrenches

% We need to set the location of the unknown wrench. We express the unknown
% wrench at the origin of the l_sole frame
unknownWrench = iDynTree.UnknownWrenchContact();
unknownWrench.unknownType = iDynTree.FULL_WRENCH;

% the position is the origin, so the conctact point wrt to l_sole is zero
unknownWrench.contactPoint.zero();

% The fullBodyUnknowns is a class storing all the unknown external wrenches
% acting on a class
fullBodyUnknowns = iDynTree.LinkUnknownWrenchContacts(estimator.model());
fullBodyUnknowns.clear();

fullBodyUnknowns.addNewContactInFrame(estimator.model(),l_sole_index,unknownWrench);

% Print the unknowns to make sure that everything is properly working
fullBodyUnknowns.toString(estimator.model())

%% Run the prediction of FT measurements

% There are three output of the estimation:

% The estimated FT sensor measurements
estFTmeasurements = iDynTree.SensorsMeasurements(estimator.sensors());

% The estimated joint torques
estJointTorques = iDynTree.JointDOFsDoubleArray(dofs);

% The estimated contact forces
estContactForces = iDynTree.LinkContactWrenches(estimator.model());

% run the estimation
estimator.computeExpectedFTSensorsMeasurements(fullBodyUnknowns,estFTmeasurements,estContactForces,estJointTorques);

% print the estimated measurements
nrOfFTSensors = estimator.sensors().getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);
for ftIndex = 0:(nrOfFTSensors-1)
    estimatedSensorWrench = iDynTree.Wrench();
    sens = estimator.sensors().getSensor(iDynTree.SIX_AXIS_FORCE_TORQUE,ftIndex);
    estFTmeasurements.getMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE,ftIndex,estimatedSensorWrench);

    % Print info
    fprintf('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n');
    fprintf('Sensor %s has index %d\n',sens.getName(),ftIndex);
    fprintf('Estimated measured wrench: %s',estimatedSensorWrench.toString());

    % the estimated sensor wrench can be easily converted to matlab with
    % estimatedSensorWrench.toMatlab()
end

% print the estimated contact forces
estContactForces.toString(estimator.model())


