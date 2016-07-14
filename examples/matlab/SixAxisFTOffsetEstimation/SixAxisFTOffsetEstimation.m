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

%% We can use the same class also for performing external wrenches estimation,
%% assuming that calibrated (i.e. without offset) F/T sensor measurements are available
%% For the sake of the example, we use the same FT measurements estimated, but
%% if actual FT sensor measurements were available we could set them in the SensorsMeasurements
%% object by calling the setMeasurements method.

% We first need a new set of unknowns, as we now need 7 unknown wrenches, one for
% each submodel in the estimator
fullBodyUnknownsExtWrenchEst = iDynTree.LinkUnknownWrenchContacts(estimator.model());

% We could fill this automatically, but in this example is interesting to have full control
% of the frames in which this wrenches are expressed (to see the link and frames of a model,
% just type idyntree-model-info -m nameOfUrdfFile.urdf -p in a terminal 

% Foot contacts
fullBodyUnknownsExtWrenchEst.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),estimator.model().getFrameIndex('l_sole'));
fullBodyUnknownsExtWrenchEst.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),estimator.model().getFrameIndex('r_sole'));

% Knee contacts
fullBodyUnknownsExtWrenchEst.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),estimator.model().getFrameIndex('l_lower_leg'));
fullBodyUnknownsExtWrenchEst.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),estimator.model().getFrameIndex('r_lower_leg'));

% Contact on the central body
fullBodyUnknownsExtWrenchEst.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),estimator.model().getFrameIndex('root_link'));

% Contacts on the hands
fullBodyUnknownsExtWrenchEst.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),estimator.model().getFrameIndex('l_elbow_1'));
fullBodyUnknownsExtWrenchEst.addNewUnknownFullWrenchInFrameOrigin(estimator.model(),estimator.model().getFrameIndex('r_elbow_1'));

% We also need to allocate the output of the estimation: a class for estimated contact wrenches and one for joint torques
% The estimated external wrenches
estContactForcesExtWrenchesEst = iDynTree.LinkContactWrenches(estimator.model());

% The estimated joint torques
estJointTorquesExtWrenchesEst = iDynTree.JointDOFsDoubleArray(dofs);

% Now we can call the estimator
estimator.estimateExtWrenchesAndJointTorques(fullBodyUnknownsExtWrenchEst,estFTmeasurements,estContactForcesExtWrenchesEst,estJointTorquesExtWrenchesEst);

% We can now print the estimated external forces : as the FT sensor measurements where estimated
% under the assumption that the only external wrench is acting on the left foot, we should see
% that the only non-zero wrench is the one on the left foot (frame: l_sole)
fprintf('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n');
fprintf('External wrenches estimated using the F/T offset computed in the previous step\n');
fprintf('%s',estContactForcesExtWrenchesEst.toString(estimator.model()));

% Wrenches values can easily be obtained as matlab vectors
estContactForcesExtWrenchesEst.contactWrench(estimator.model().getLinkIndex('l_foot'),0).contactWrench().getLinearVec3().toMatlab()


% LinkContactWrenches is a structure that can contain multiple contact wrench for each link,
% but usually is convenient to just deal with a collection of net wrenches for each link
linkNetExtWrenches = iDynTree.LinkWrenches(estimator.model())
estContactForcesExtWrenchesEst.computeNetWrenches(linkNetExtWrenches);

% also net external wrenches can easily be obtained as matlab vectors
wrench = linkNetExtWrenches(estimator.model().getLinkIndex('l_foot'));
% 6d wrench (force/torques)
wrench.toMatlab()
% just the force
wrench.getLinearVec3().toMatlab()




