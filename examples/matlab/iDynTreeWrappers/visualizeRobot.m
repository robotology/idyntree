%% This script is an example on how to use the matlab iDynTree visualization
% To use insert the path where the meshes may be found, it replaces what is required
% to complete the path inside the urdf visual mesh field.

% Example:
% If a test model has the following mesh file in the URDF
% 'package://stl/sim_sea_2-5_root_link_prt-binary.stl'
% The required path is what it takes to reach the stl folder.
% meshFilePrefix=<path_to_stl_folder>;
% It basically replaces the functionality of find Package by inserting the
% path manually.
% The path to the model is also required:
% modelPath=<path_to_urdf_folder>;

% REMARK : If you have installed the URDF models by https://github.com/robotology/icub-models
% You could fill the required variables as follows:
% % Substitute in the following the location of the install prefix of icub-models
icubModelsInstallPrefix = '';

% if installed with robotology superbuild you can directly use
icubModelsInstallPrefix = getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX');


 meshFilePrefix = [icubModelsInstallPrefix '/share'];
% Select the robot using the folder name
robotName='';

% Example
robotName='iCubGenova04';

 modelPath = [icubModelsInstallPrefix '/share/iCub/robots/' robotName '/'];
 fileName='model.urdf';

 consideredJoints={
    'r_hip_pitch';
    'r_hip_roll';
    'r_hip_yaw';
    'r_knee';
    'r_ankle_pitch';
    'r_ankle_roll';
    'l_hip_pitch';
    'l_hip_roll';
    'l_hip_yaw';
    'l_knee';
    'l_ankle_pitch';
    'l_ankle_roll';
    'torso_pitch';
    'torso_roll';
    'torso_yaw';
    'r_shoulder_pitch';
    'r_shoulder_roll';
    'r_shoulder_yaw';
    'r_elbow';
    'r_wrist_prosup';
    'r_wrist_pitch';
    'r_wrist_yaw';
    'l_shoulder_pitch';
    'l_shoulder_roll';
    'l_shoulder_yaw';
    'l_elbow';
    'l_wrist_prosup';
    'l_wrist_pitch';
    'l_wrist_yaw';
    'neck_pitch';
    'neck_roll';
    'neck_yaw'
    };

% Main variable of iDyntreeWrappers used for many things including updating
% robot position and getting world to frame transforms
KinDynModel = iDynTreeWrappers.loadReducedModel(consideredJoints,'root_link',modelPath,fileName,false);

% create vector of positions
joints_positions=zeros(KinDynModel.NDOF,1);

% add a world to base mainly to avoid overlap of coordinate frame and robot
world_H_base=[1,0,0,0;0,1,0,0;0,0,1,0.6;0,0,0,1];

% Set initial position of the robot
iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joints_positions,zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);

% Prepare figure, handles and variables required for the update, some extra
% options are commented.
[visualizer,objects]=iDynTreeWrappers.prepareVisualization(KinDynModel,meshFilePrefix,...
    'color',[0,0,1],'material','metal','transparency',1,'debug',true,'view',[-92.9356   22.4635],...
    'groundOn',true,'groundColor',[0.5 0.5 0.5], 'groundTransparency',0.5,'groundFrame','l_sole');%,... % optional inputs
     %'style','wireframe','wireframe_rendering',0.1);

% pause to let you see the result with the options that are enabled
pause(1);
% generate decent position
joints_positions=zeros(KinDynModel.NDOF,1);
joints_positions(KinDynModel.kinDynComp.model.getJointIndex('r_shoulder_roll')) = pi/10;
joints_positions(KinDynModel.kinDynComp.model.getJointIndex('r_shoulder_yaw')) = pi/10;
joints_positions(KinDynModel.kinDynComp.model.getJointIndex('r_elbow')) = pi/10;
joints_positions(KinDynModel.kinDynComp.model.getJointIndex('l_shoulder_roll')) = pi/10;
joints_positions(KinDynModel.kinDynComp.model.getJointIndex('l_shoulder_yaw')) = pi/10;
joints_positions(KinDynModel.kinDynComp.model.getJointIndex('l_elbow')) = pi/10;

%% Update robot position
% update kinematics
iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joints_positions,zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);

tic
 iDynTreeWrappers.updateVisualization(KinDynModel,visualizer);
stringVector_time=toc
axis tight

%% Use modification functions
% Change all links to default values
iDynTreeWrappers.modifyLinksVisualization(visualizer,'useDefault',true);

% Modify using link indices
indecesToModify = find(contains(visualizer.linkNames,'head'));
iDynTreeWrappers.modifyLinksVisualization(visualizer,'linksIndices',indecesToModify,'color',[1,0,0]);

indecesToModify=[find(contains(visualizer.linkNames,'l_forearm')), ...
                 find(contains(visualizer.linkNames,'l_hand'))];
iDynTreeWrappers.modifyLinksVisualization(visualizer,'linksIndices',indecesToModify,'color',[0,0,0],'transparency',1,'material','metal');

% Modify using link names
% Select link names to modify
linkstoModify = [{'r_upper_leg'}, {'r_lower_leg'}, {'r_ankle_1'}, {'r_ankle_2'}];

iDynTreeWrappers.modifyLinksVisualization(visualizer,'linksToModify',linkstoModify,'style','wireframe');
pause(1);
iDynTreeWrappers.modifyLinksVisualization(visualizer,'linksToModify',linkstoModify(3),'style','invisible');
pause(1);
iDynTreeWrappers.modifyLinksVisualization(visualizer,'linksToModify',linkstoModify(3),'style','fullMesh');
