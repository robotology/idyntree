function [Visualizer,Objects]=prepareVisualization(KinDynModel,meshFilePrefix,varargin)
% Creates the figures, loads the meshes, handles some visual effects, and creates the transform objects
% - Inputs:
%   - `KinDynModel` : iDyntreewrappers main variable. Contains the model.
%   - `meshFilePrefix` : Path in which we can find the meshes. As an example the path to the mesh in a iCub urdf is `'package://iCub/meshes/simmechanics/sim_sea_2-5_root_link_prt-binary.stl'
%   `. `meshFilePrefix` should replace package to allow to find the rest of the path.
% - Optional Inputs:
%     - `view` : Selects the angle in which the figure is seen.
%     - `material` : Selects effect with which the patch is rendered. Options are : 'dull','metal','shiny';
%     - `debug` : Enables having extra information for debugging purposes.
%     - `groundOn` : Enables showing a plane as the ground.
%     - `groundColor` : Selects the color of the ground.
%     - `groundTransparency` : Selects the transparency of the ground.
%     - `groundFrame` : Selects the frame in which the ground is attached.
%     - `name` : The name of the figure 
%     - `reuseFigure` : Enable the reuse of an already open figure. It can be the following values:
%         - 'name': Reuse the figure with the same name (the figure is cleared before reusing it). The name must be set.
%         - 'gcf': Reuse the figure returned by gcf.
%         - 'none': Do not reuse the figure (Default).
%     Note: all extra variables are sent to `plotMeshInWorld`
%   - Outputs:
%       - `Visualizer` : Struct containing the following fields
%           - `transforms` : Is the transform objects array. There is one transform for each link
%           - `linkNames`  : The name of the links in the same order of the transforms
%           - `linkNames_idyn`  : The idyntree string vector equivalent to `linkNames`
%           - `NOBJ` : Contains the number of visual objects in the visualizer
%           - `meshHandles` : Has the handles of the mesh objects.
%           - `parent`     : Contains the axes object of the parent figure.
%           - `mainHandler`: Is the handle of the overall figure.
%           - `DEBUG` : Flag stating if the debug mode was set or not.
%           Fields included if debug mode is on:
%           - `map`       : Cell array having both the names of the meshes and the associated link
%           - `linkMeshInfo` : Contains the link name and a struct (meshInfo) that contains the name of file or if is a simple geometry, the triangulation ( edges and vertices of the mesh ) and the link to geometry transform.% Author : Francisco Andrade (franciscojavier.andradechavez@iit.it)
%
% Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.
%% input parser section
p = inputParser;
p.StructExpand = false;
p.KeepUnmatched= true;
% Default values
default_view=[128.0181 16.8000];
default_material='dull';
default_debug=false;
default_groundOn=false;
default_groundColor=[0 0.5 0.5];
default_groundTransparency=0.8;
default_groundFrame='none';
default_name='iDynTreeVisualizer';
default_reuseFigure='none';
% accepted values
expected_materials={'dull','metal','shiny'};
expected_reuseFigure={'name', 'gcf', 'none'};
% add parameters and validity funcitons
addRequired(p,'structInput');
addRequired(p,'meshFilePrefix',@(x) isstring(x) || ischar(x));
addParameter(p,'view',default_view,@isnumeric);
addParameter(p,'material',default_material,@(x) any(validatestring(x,expected_materials)));
addParameter(p,'debug',default_debug,@(x) islogical(x));
addParameter(p,'groundOn',default_groundOn,@(x) islogical(x));
addParameter(p,'groundColor',default_groundColor,@(x)validateattributes(x,{'numeric'},{'numel', 3}));
addParameter(p,'groundTransparency',default_groundTransparency,@(x) isnumeric(x) && isscalar(x));
addParameter(p,'groundFrame',default_groundFrame,@(x) isstring(x) || ischar(x));
addParameter(p,'name',default_name,@(x) isstring(x) || ischar(x));
addParameter(p,'reuseFigure',default_reuseFigure,@(x) any(validatestring(x,expected_reuseFigure)));

% parse inputs
parse(p,KinDynModel,meshFilePrefix,varargin{:});
options=p.Results;
isNameSet=~any(strcmp(p.UsingDefaults, 'name'));
%% Get meshes from the model variable
model=KinDynModel.kinDynComp.model;
[linkMeshInfo,map]=iDynTreeWrappers.getMeshes(model,meshFilePrefix);
numberOfLinks=length(linkMeshInfo);
linkNames=cell(numberOfLinks,1);
switch options.reuseFigure
    case 'gcf'
        mainHandler=gcf;
        cla(mainHandler.Children);
    case 'name'
        figHandles = findobj('Type', 'figure', 'Name', options.name);
        if isNameSet && ~isempty(figHandles)
            mainHandler=figHandles(1,1);
            cla(mainHandler.Children);
        else
            mainHandler=figure;
        end
    otherwise
        mainHandler=figure;
end
if isNameSet
    set(mainHandler,'Name', options.name,'numbertitle','off')
end
set(0, 'CurrentFigure', mainHandler) %Set the figure as current figure such that gca works
parent=gca;
hold on
linkNames_idyn=iDynTree.StringVector();

for it=1:numberOfLinks
    w_H_link=iDynTreeWrappers.getWorldTransform(KinDynModel,linkMeshInfo(it).linkName);
    [meshHandles(it,:),transforms(it)]=iDynTreeWrappers.plotMeshInWorld(linkMeshInfo(it),w_H_link,varargin{:});
    linkNames{it}=linkMeshInfo(it).linkName;
    linkNames_idyn.push_back(linkNames{it});
end

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material(options.material);

% Make axis pretty and add grid for visual reference
axis equal;
axis tight;
grid on;

% Apply views
view(parent,options.view);

% Add world reference frame
hold on;
currentAxisValues=axis;
axisRange=currentAxisValues([2,4,6])-currentAxisValues([1,3,5]);
frameAxisSize = min(axisRange)/4;
% linewidth value 1 = to 0.35mm. It is better if we increase the linewidth
% in proportion to the axisSize of the frame.c
linewidthSize=frameAxisSize*50;
plot3(parent, [0 frameAxisSize], [0 0], [0 0], 'r', 'linewidth', linewidthSize);
plot3(parent, [0 0], [0 frameAxisSize], [0 0], 'g', 'linewidth', linewidthSize);
plot3(parent, [0 0], [0 0], [0 frameAxisSize], 'b', 'linewidth', linewidthSize);
axisTextDistance = frameAxisSize+frameAxisSize*.2;
text(parent, axisTextDistance, 0, 0, 'x', 'color', 'r');
text(parent, 0, axisTextDistance, 0, 'y', 'color', 'g');
text(parent, 0, 0, axisTextDistance, 'z', 'color', 'b');

% Axis labels
title('Robot Visualizer ');
xlabel('{x}');
ylabel('{y}');
zlabel('{z}');

% Draw the ground
if options.groundOn
    % Create plane on x y axis
    normal_vector=[0,0,1];
    point_on_plane=[0,0,0];
    d=normal_vector*point_on_plane';
    x = [1 -1 -1 1]*max(axisRange(1:2)); % Generate data for x vertices
    y = [1 1 -1 -1]*max(axisRange(1:2)); % Generate data for y vertices
    z = 1/normal_vector(3)*(normal_vector(1)*x + normal_vector(2)*y + d); % Solve for z vertices data
    groundHandle=patch('XData',x,'YData',y,'ZData',z,'FaceColor', options.groundColor);
    alpha(groundHandle,options.groundTransparency);
    % apply transform
    groundTransform = hgtransform('Parent',parent);
    set(groundHandle,'Parent',groundTransform);
    if any(ismember(p.UsingDefaults,'groundFrame'))
        w_H_plane=eye(4,4);
        options.groundFrame=linkNames{1};
    else
        w_H_plane_idyn=KinDynModel.kinDynComp.getWorldTransform(options.groundFrame);
        w_H_plane= w_H_plane_idyn.asHomogeneousTransform.toMatlab();
        w_H_plane(1:2,4)=0;
    end
    set(groundTransform,'Matrix',w_H_plane);

    objectFrames_idyn=iDynTree.StringVector();
    objectFrames_idyn.push_back(options.groundFrame);

    % Build Object struct
    Objects.frames_idyn=objectFrames_idyn;
    Objects.transform=groundTransform;
    Objects.frames=options.groundFrame;
    Objects.types={'plane'};
    Objects.names={'ground'};
    Objects.handle=groundHandle;

    if options.debug
        Objects.map=[Objects.names',Objects.frames'];
    end
else
    Objects=[];
end

% Build Output struct
Visualizer.transforms=transforms;
Visualizer.linkNames_idyn=linkNames_idyn;
Visualizer.linkNames=linkNames;
Visualizer.NOBJ=length(transforms);
Visualizer.meshHandles=meshHandles;
Visualizer.mainHandler=mainHandler;
Visualizer.parent=parent;
Visualizer.DEBUG=options.debug;

if options.debug
    Visualizer.map=map;
    Visualizer.linkMeshInfo=linkMeshInfo;
end
