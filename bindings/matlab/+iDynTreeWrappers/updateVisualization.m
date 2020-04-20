function []=updateVisualization(KinDynModel,linkNames,transforms)
% We use the transform handlers to update the robot image using the
% iDyntree kinematics
% Updates the figure image with the new robot state. To be launched after `setRobotState` has been used.
%     - Inputs:
%           - `KinDynModel`: iDyntreewrappers main variable. Contains the model.
%           - `linkNames`  : The `linkNames` variable output from the `prepareVisualization` function that contains the link names.
%           - `transforms` : The `transforms` variable output from the `prepareVisualization` function.
%
% Author : Francisco Andrade (franciscojavier.andradechavez@iit.it)
%
% Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.

if iscellstr(linkNames) % sending an cell array with the link names
    for it=1:length(linkNames)
        w_H_link=iDynTreeWrappers.getWorldTransform(KinDynModel,linkNames{it});
        transforms(it).Matrix=w_H_link;
    end    
else % expecting an iDyntree.StringVector. This is more time efficient
    transforms_idyn=KinDynModel.kinDynComp.getWorldTransformsAsHomogeneous(linkNames);
    w_H_links= transforms_idyn.toMatlab();
    for it=1:KinDynModel.kinDynComp.getNrOfLinks
        transforms(it).Matrix=squeeze(w_H_links(it,:,:));
    end
end
