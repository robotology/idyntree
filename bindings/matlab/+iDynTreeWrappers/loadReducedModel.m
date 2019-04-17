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
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    disp(['[loadReducedModel]: loading the following model: ',[modelPath,modelName]]);
        
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

    modelLoader.loadReducedModelFromFile([modelPath,modelName], jointList_idyntree);

    % get the number of degrees of freedom of the reduced model
    KinDynModel.NDOF       = reducedModel.getNrOfDOFs();

    % initialize the iDyntree KinDynComputation class, that will be used for
    % computing the floating base system state, dynamics, and kinematics
    KinDynModel.kinDynComp = iDynTree.KinDynComputations();

    KinDynModel.kinDynComp.loadRobotModel(reducedModel);
    
    % set the floating base link
    KinDynModel.kinDynComp.setFloatingBase(KinDynModel.BASE_LINK);
    
    disp(['[loadReducedModel]: loaded model: ',[modelPath,modelName],', number of joints: ',num2str(KinDynModel.NDOF)]);
end
