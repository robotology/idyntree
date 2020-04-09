function [transforms,linkNames,linkNames_idyn,map,linkMeshInfo,meshHandles,parent,mainHandler]=prepareVisualization(KinDynModel,meshFilePrefix)
% Creates the figures, loads the meshes, handles some visual effects, and creates the transform objects
% - Iputs:
%   - `KinDynModel` : iDyntreewrappers main variable. Contains the model.
%   - `meshFilePrefix` : Path in which we can find the meshes. As an example the path to the mesh in a iCub urdf is `'package://iCub/meshes/simmechanics/sim_sea_2-5_root_link_prt-binary.stl'
%   `. `meshFilePrefix` should replace package to allow to find the rest of the path.
% - Outputs:
%   - `transforms` : Is the transform objects array. There is one transform for each link
%   - `linkNames`  : The name of the links in the same order of the transforms
%   - `linkNames_iDyn`  : The name of the links in the same order of the
%   transforms in a iDyntree string vector variable
%   - `map`       : Cell array having both the names of the meshes and the associated link
%   - `linkMeshInfo` : Contains the link name and a struct (meshInfo) that contains the name of file or if is a simple geometry, the triangulation ( edges and vertices of the mesh ) and the link to geometry transform.
%   - `meshHandles` : Has the handles of the mesh objects.
%   - `parent`     : Contains the axes object of the parent figure.
%   - `mainHandler`: Is the handle of the overall figure.
%
% Author : Francisco Andrade (franciscojavier.andradechavez@iit.it)
%
% Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.

% Desirable views
custom_view=[128.0181 16.8000];
custom_view2=[43.9924 18.0000];
% Get meshes from the model variable
model=KinDynModel.kinDynComp.model;
[linkMeshInfo,map]=iDynTreeWrappers.getMeshes(model,meshFilePrefix);
numberOfLinks=length(linkMeshInfo);
linkNames=cell(numberOfLinks,1);
mainHandler=figure;
parent=gca;
hold on
linkNames_idyn=iDynTree.StringVector();

for it=1:numberOfLinks
    w_H_link=iDynTreeWrappers.getWorldTransform(KinDynModel,linkMeshInfo(it).linkName);
    [meshHandles(it,:),transforms(it)]=iDynTreeWrappers.plotMeshInWorld(linkMeshInfo(it),w_H_link);
    linkNames{it}=linkMeshInfo(it).linkName;
    linkNames_idyn.push_back(linkNames{it});
end

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');

% Make axis pretty and add grid for visual reference
axis equal;
grid on;

% Axis labels
title('Robot Visualizer ');
xlabel('{x}');
ylabel('{y}');
zlabel('{z}');

% Apply views
view(parent,custom_view);
