%% In this example, we use the iDynTree.Model and the iDynTree.KinDynComputations
% classes to express the joint axis for a set of revolute axis in the base
% frame

% Create KinDynComputations class
comp = iDynTree.KinDynComputations();

% Load model from an URDF file
comp.loadRobotModelFromFile('../models/iCubGenova02.urdf');

% Check if the model was correctly created by printing the model
comp.getRobotModel().toString()

%% Set kinematics information

% For computing the forward kinematics, we need to set the robot position 
% and velocity with respect to an inertial frame and the joint positions 
% and velocities. In this case we will just put the joint in the 0 position
% and the world_H_base to identity and all the velocities to 0 

robotState = {};
robotState.world_H_base = iDynTree.Transform.Identity();
robotState.base_vel     = iDynTree.Twist.Zero();
% The joint pos/vel vectors are automatically resize with a constructor 
% that takes in input the robot model 
robotState.qj           = iDynTree.VectorDynSize();
robotState.qj.resize(comp.getRobotModel().getNrOfPosCoords());
robotState.qj.zero();
robotState.dqj          = iDynTree.VectorDynSize();
robotState.dqj.resize(comp.getRobotModel().getNrOfDOFs());
robotState.dqj.zero();
% Gravity expressed in the world frame 
robotState.gravity      = iDynTree.Vector3();
robotState.gravity.fromMatlab([0,0,-9.81]);

% Actually set the state in the kinDynComp class (we use the setted that
% does not requires to set the base variables and assume that they are 
% the identity and zero, because the full one for some reason does not
% works)
% comp.setRobotState(robotState.world_H_base,robotState.qj,robotState.base_vel,robotState.dqj,robotState.gravity);
comp.setRobotState(robotState.qj,robotState.dqj,robotState.gravity);

%% Compute relative transform between frames 

% To compute the transform f1_H_f2 between two frames f1, f2,
% you can just call the getRelativeTransform 
% Note: a list of available frames can be obtained by printing the model 
root_link_H_l_sole = comp.getRelativeTransform('root_link','l_sole');


root_link_H_l_sole.asHomogeneousTransform.toMatlab 

% To compute the transform world_H_f1, just use the getWorldTransform
% method: 
world_H_l_sole = comp.getWorldTransform('l_sole');

world_H_l_sole.asHomogeneousTransform.toMatlab 
% this should be equal to world_H_l_sole because we set the world_H_base 
% to identity, and root_link is the base link of this model 


%% Compute axis in the world frame 
model = comp.getRobotModel();

