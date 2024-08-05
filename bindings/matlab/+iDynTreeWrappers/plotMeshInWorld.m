function [meshHandles,transform]=plotMeshInWorld(linkMeshInfo,w_H_link,varargin)
    % Gets the mesh information for each link in the model.
    %     - Iputs:
    %         - `linkMeshInfo` : An individual `linkMeshInfo` from the array output variable from `getMeshes` function.
    %         - `w_H_link` : Homogeneous transform from the world to the link
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
    % accepted values
    expected_styles={'fullMesh','wireframe'};
    % add parameters and validity funcitons
    addRequired(p,'structInput');
    addRequired(p,'w_H_link',@(x)validateattributes(x,{'numeric'},{'size',[4,4]}));
    addParameter(p,'color',default_color,@(x)validateattributes(x,{'numeric'},{'numel', 3}));
    addParameter(p,'transparency',default_transparency,@(x) isnumeric(x) && isscalar(x));
    addParameter(p,'wireframe_rendering',default_wireframe_rendering,@(x) isnumeric(x) && isscalar(x));
    addParameter(p,'style',default_style,@(x) any(validatestring(x,expected_styles)));
    % parse inputs
    parse(p,linkMeshInfo,w_H_link,varargin{:});
    options=p.Results;

    %% Create a transform object
    transform = hgtransform('Parent',gca);

    for mesh_number = 1:length(linkMeshInfo.meshInfo)

        mesh_triangles = linkMeshInfo.meshInfo(mesh_number).mesh_triangles;
        link_H_geom = linkMeshInfo.meshInfo(mesh_number).link_H_geom;
        scale = linkMeshInfo.meshInfo(mesh_number).scale;

        % Scale the STL mesh
        corrected_vertices = mesh_triangles.Points.*scale;

        % Applying transform to link frame to stl file:
        corrected_vertices = link_H_geom*[corrected_vertices';ones(1,size(corrected_vertices,1))];
        corrected_vertices = corrected_vertices';
        corrected_vertices = corrected_vertices(:,1:3);

        % Plot in the figure using patch
        modelMesh(mesh_number) = patch('Faces',mesh_triangles.ConnectivityList, ...
            'Vertices',corrected_vertices, ...
            'FaceColor',options.color , ...
            'EdgeColor','none', ...
            'FaceLighting','gouraud', ...
            'AmbientStrength', 0.25);

        faces_bckup = modelMesh(mesh_number).Faces;
        vertices_bckup = modelMesh(mesh_number).Vertices;

        meshHandles.fullMesh_bckup(mesh_number).faces = faces_bckup;
        meshHandles.fullMesh_bckup(mesh_number).vertices = vertices_bckup;

        if strcmp(options.style,'wireframe')
            reducepatch(modelMesh(mesh_number),options.wireframe_rendering);
            modelMesh(mesh_number).FaceColor = 'none';
            modelMesh(mesh_number).EdgeColor = options.color;
        end

        % Parent transform as parent to the mesh
        set(modelMesh(mesh_number),'Parent',transform);

        % Set the object transparency
        alpha(modelMesh(mesh_number),options.transparency);
    end

    set(transform,'Matrix',w_H_link);
    meshHandles.modelMesh = modelMesh;
end
