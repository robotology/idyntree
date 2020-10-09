function [] = updateVisualization(KinDynModel, Visualizer)

    % UPDATEVISUALIZATION uses the transform handlers to update the robot
    %                     image using the iDyntree kinematics.
    %
    % NOTE: to be launched after setRobotState has been used.
    %
    % FORMAT [] = updateVisualization(KinDynModel, Visualizer)
    %
    % INPUTS:  - KinDynModel: iDyntreewrappers main variable. Contains the model.
    %          - Visualizer:  variable output from the prepareVisualization function. 
    %                         It contains the relevant variables
    %                         linkNames,transforms and NOBJ:
    %
    %              * linkNames: variable that contains the link names.
    %              * transforms: contains transform objects that are parent of the meshes.
    %              * NOBJ: variable that contains the number of visual objects to update.
    %
    % Author : Francisco Andrade (franciscojavier.andradechavez@iit.it)
    %          Modified by: Gabriele Nava (gabriele.nava@iit.it)
    %    
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    transforms_idyn = KinDynModel.kinDynComp.getWorldTransformsAsHomogeneous(Visualizer.linkNames_idyn);
    w_H_links       = transforms_idyn.toMatlab();
    
    for it = 1:Visualizer.NOBJ
     
        Visualizer.transforms(it).Matrix = squeeze(w_H_links(it,:,:));
    end
end
