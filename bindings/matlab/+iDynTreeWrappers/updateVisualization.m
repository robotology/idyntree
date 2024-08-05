function []=updateVisualization(KinDynModel,Visualizer)
    % We use the transform handlers to update the robot image using the
    % iDyntree kinematics
    % Updates the figure image with the new robot state. To be launched after `setRobotState` has been used.
    %     - Inputs:
    %           - `KinDynModel`: iDyntreewrappers main variable. Contains the model.
    %           - `visualizer` : variable output from the `prepareVisualization` function. It contains the relevant variables linkNames,transforms and NOBJ
    %               - `linkNames`  : variable that contains the link names.
    %               - `transforms` : variable that contains transform objects that are parent of the meshes.
    %               - `NOBJ` : variable that contains the number of visual objects to update.
    %
    % Author : Francisco Andrade (franciscojavier.andradechavez@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------
    transforms_idyn = KinDynModel.kinDynComp.getWorldTransformsAsHomogeneous(Visualizer.linkNames_idyn);
    w_H_links = transforms_idyn.toMatlab();

    for it = 1:Visualizer.NOBJ

        Visualizer.transforms(it).Matrix = squeeze(w_H_links(it,:,:));
    end
end
