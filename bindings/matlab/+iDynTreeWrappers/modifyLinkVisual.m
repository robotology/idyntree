function []=modifyLinkVisual(meshHandle,varargin)
    % Modifies a single `meshHandle` with the specified changes.
    %     - Inputs:
    %         - `meshHandle` : Struct containing the patch type of variable ( `modelMesh` field ) and the original information of the mesh in `fullMesh_bckup` field.
    %     - Optional Inputs:
    %         - `color` : Selects the color of the meshes.
    %         - `transparency` : Sets the alpha value of the patch. Is the
    %         level of transparency between 0 fully transparent and 1 solid.
    %         - `wireframe_rendering` : the reduction ratio of faces to
    %         wireframe. Follows convention of reducepatch.
    %         - `style` : Selects the style of display of meshes, either fullMesh or wireframe.
    %     - Outputs:
    %         - `meshHandles`  : Struct that contains all the created handles for the meshes. There can be more than one mesh for each link.
    %         - `transform`    : The transform object for the link
    %         - `material` : Selects effect with which the patch is rendered. Options are : 'dull','metal','shiny';
    %         - `useDefault`  : Enables the use of the default values instead of ignoring non specified changes.
    %
    % Author : Francisco Andrade (franciscojavier.andradechavez@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause
    %% input parser section
    p = inputParser;
    p.StructExpand = false;
    p.KeepUnmatched= true;
    % Default values
    default_color= [0.1843    0.3098    0.3098];
    default_transparency=0.5;
    default_style='fullMesh';
    default_wireframe_rendering=0.08;
    default_material='dull';
    default_useDefault=false;
    % accepted values
    expected_materials={'dull','metal','shiny'};
    expected_styles={'fullMesh','wireframe','invisible'};
    % add parameters and validity funcitons
    addRequired(p,'structInput');
    addParameter(p,'color',default_color,@(x)validateattributes(x,{'numeric'},{'numel', 3}));
    addParameter(p,'transparency',default_transparency,@(x) isnumeric(x) && isscalar(x));
    addParameter(p,'wireframe_rendering',default_wireframe_rendering,@(x) isnumeric(x) && isscalar(x));
    addParameter(p,'style',default_style,@(x) any(validatestring(x,expected_styles)));
    addParameter(p,'material',default_material,@(x) any(validatestring(x,expected_materials)));
    addParameter(p,'useDefault',default_useDefault);

    % Parse inputs
    parse(p,meshHandle,varargin{:});
    options = p.Results;

    for nMeshes = 1:length(meshHandle.modelMesh)
        modelMesh = meshHandle.modelMesh(nMeshes);

        if ~any(ismember(p.UsingDefaults,'color')) || options.useDefault
            changeColor(modelMesh,options.color);
        end
        if ~any(ismember(p.UsingDefaults,'transparency')) || options.useDefault
            alpha(modelMesh,options.transparency);
        end
        if ~any(ismember(p.UsingDefaults,'material')) || options.useDefault
            material(modelMesh,options.material);
        end
        if ~any(ismember(p.UsingDefaults,'style')) || options.useDefault

            [isWF,noColor] = isWireframe(modelMesh);

            if strcmpi(options.style,'wireframe') && ~isWF
                if noColor
                    color = options.color;
                else
                    color = modelMesh.FaceColor;
                end
                reducepatch(modelMesh,options.wireframe_rendering);
                modelMesh.FaceColor = 'none';
                modelMesh.EdgeColor = color;
            end
            if strcmpi(options.style,'fullMesh') && isWF

                if noColor
                    color = options.color;
                else
                    color = modelMesh.EdgeColor;
                end

                modelMesh.Vertices = meshHandle.fullMesh_bckup(nMeshes).vertices;
                modelMesh.Faces = meshHandle.fullMesh_bckup(nMeshes).faces;
                modelMesh.FaceColor = color;
                modelMesh.EdgeColor = 'none';
            end
            if strcmpi(options.style,'invisible')
                % make invisible
                modelMesh.Visible = 'off';
            else
                modelMesh.Visible = 'on';
            end
        end
    end
end

% Utilities
function [] = changeColor(modelMesh,color)

    if isWireframe(modelMesh)
        modelMesh.EdgeColor = color;
    else
        modelMesh.FaceColor = color;
    end
end

function [isIt, noColor] = isWireframe(meshHandle)

    % If there is a numeric value in edges and a char ('none') in faces
    % then is wireframe
    noColor = false;

    if ischar(meshHandle.FaceColor) && ~ischar(meshHandle.EdgeColor)
        isIt = true;
    end

    % As long as there is a color in faces is not wireframe
    if ~ischar(meshHandle.FaceColor) && ischar(meshHandle.EdgeColor)
        isIt = false;
    end
    % The mesh is invisble and not on purpose. Make it wireframe.
    if ~exist('isIt','var')
        isIt = true;
        noColor = true;
    end
end
