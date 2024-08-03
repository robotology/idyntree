function KinDynModel = loadReducedModel(jointList,baseLinkName,modelPath,modelName,debugMode)

    % LOADREDUCEDMODEL loads the urdf model of the rigid multi-body system.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  KinDynModel = loadReducedModel(jointList,baseLinkName,modelPath,modelName,debugMode)
    %
    % INPUTS:  - jointList: cell array containing the list of joints to be used
    %                       in the reduced model;
    %          - baseLinkName: a string that specifies link which is considered
    %                          as the floating base;
    %          - modelPath: a string that specifies the path to the urdf model;
    %          - modelName: a string that specifies the model name;
    %          - debugMode: if TRUE, the wrappers are used in "debug" mode;
    %
    % OUTPUTS: - KinDynModel: a structure containing the loaded model and additional info.
    %
    % Author: Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------
    disp(['[loadReducedModel]: loading the following model: ',fullfile(modelPath,modelName)]);

    % if DEBUG option is set to TRUE, all the wrappers will be run in debug
    % mode. Wrappers concerning iDyntree simulator have their own debugger
    KinDynModel.DEBUG      = debugMode;

    % retrieve the link that will be used as the floating base
    KinDynModel.BASE_LINK  = baseLinkName;

    % load the list of joints to be used in the reduced model
    jointList_idyntree     = iDynTree.StringVector();

    for k = 1:length(jointList)

        jointList_idyntree.push_back(jointList{k});
    end

    % only joints specified in the joint list will be considered in the model
    modelLoader            = iDynTree.ModelLoader();
    reducedModel           = modelLoader.model();

    modelLoader.loadReducedModelFromFile(fullfile(modelPath,modelName), jointList_idyntree);

    % get the number of degrees of freedom of the reduced model
    KinDynModel.NDOF       = reducedModel.getNrOfDOFs();

    % initialize the iDyntree KinDynComputation class, that will be used for
    % computing the floating base system state, dynamics, and kinematics
    KinDynModel.kinDynComp = iDynTree.KinDynComputations();

    KinDynModel.kinDynComp.loadRobotModel(reducedModel);

    % set the floating base link
    KinDynModel.kinDynComp.setFloatingBase(KinDynModel.BASE_LINK);

    disp(['[loadReducedModel]: loaded model: ',fullfile(modelPath,modelName),', number of joints: ',num2str(KinDynModel.NDOF)]);

    % initialize all dynamics and kinematics quantities used inside the
    % wrappers. This procedure optimizes the wrappers speed, as the
    % iDyntree objects are created only once and then updated runtime
    KinDynModel.kinematics.baseRotation_iDyntree = iDynTree.Rotation();
    KinDynModel.kinematics.baseOrigin_iDyntree = iDynTree.Position();
    KinDynModel.kinematics.basePose_iDyntree = iDynTree.Transform();
    KinDynModel.kinematics.baseVel_iDyntree = iDynTree.Twist();
    KinDynModel.kinematics.jointPos_iDyntree = iDynTree.VectorDynSize(KinDynModel.NDOF);
    KinDynModel.kinematics.jointVel_iDyntree = iDynTree.VectorDynSize(KinDynModel.NDOF);
    KinDynModel.kinematics.stateVel_iDyntree = iDynTree.VectorDynSize(6+KinDynModel.NDOF);
    KinDynModel.kinematics.gravityVec_iDyntree = iDynTree.Vector3();
    KinDynModel.kinematics.J_CoM_iDyntree = iDynTree.MatrixDynSize(3,KinDynModel.NDOF+6);
    KinDynModel.kinematics.J_frame_iDyntree = iDynTree.MatrixDynSize(6,KinDynModel.NDOF+6);
    KinDynModel.kinematics.J_frameVel_iDyntree = iDynTree.MatrixDynSize(6,KinDynModel.NDOF);
    KinDynModel.dynamics.M_iDyntree = iDynTree.MatrixDynSize(KinDynModel.NDOF+6,KinDynModel.NDOF+6);
    KinDynModel.dynamics.h_iDyntree = iDynTree.FreeFloatingGeneralizedTorques(KinDynModel.kinDynComp.model);
    KinDynModel.dynamics.g_iDyntree = iDynTree.FreeFloatingGeneralizedTorques(KinDynModel.kinDynComp.model);
end
