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
 meshFilePrefix = [icubModelsInstallPrefix '/share'];
% Select the robot using the folder name
 robotName='';
 modelPath = [icubModelsInstallPrefix '/share/iCub/robots/' robotName '/'];
 fileName='model.urdf';

jointOrder={
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
%
KinDynModel = iDynTreeWrappers.loadReducedModel(jointOrder,'root_link',modelPath,fileName,false);

% create vector of positions
joints_positions=zeros(KinDynModel.NDOF,1);

% Prepare figure, handles and variables required for the update
[transforms,linkNames,linkNames_idyn,map,linkMeshInfo,meshHandles,parent,mainHandler]=iDynTreeWrappers.prepareVisualization(KinDynModel,meshFilePrefix);
% Note: For default visualization these variables can be ignored:
%       map,linkMeshInfo,meshHandles,parent,mainHandler.

% generate random positions for the robot
a = -pi/2;
b = pi/2;
randVector=(b-a).*rand(KinDynModel.NDOF,1)+ a;

% update only joints of arms and head to avoid ackward positions.
joints_positions(16:end)=randVector(16:end);

% update kinematics
iDynTreeWrappers.setRobotState(KinDynModel,eye(4),joints_positions,zeros(6,1),zeros(size(joints_positions)),[0,0,-9.81]);

%%  Compare time efficiency of the update function
% using iDyntree string vector
tic
 iDynTreeWrappers.updateVisualization(KinDynModel,linkNames_idyn,transforms);
stringVector_time=toc

% using a string cell array with the link names
tic
 iDynTreeWrappers.updateVisualization(KinDynModel,linkNames,transforms);
cellArray_time=toc

axis tight

%% Try some custom changes
toModify=meshHandles(1).modelMesh;
% wireFrame style
color=toModify.FaceColor;
toModify.FaceColor='none';
toModify.EdgeColor=color;
faces=toModify.Faces;
vertices=toModify.Vertices;
reducepatch(toModify,0.005)

% go back to normal
toModify.Vertices=vertices;
toModify.Faces=faces;
toModify.FaceColor=color;
toModify.EdgeColor='none';
