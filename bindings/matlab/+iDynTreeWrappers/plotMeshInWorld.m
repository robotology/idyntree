function [meshHandles,transform]=plotMeshInWorld(linkMeshInfo,w_H_link)
% Gets the mesh information for each link in the model.
%     - Iputs:
%         - `linkMeshInfo` : An individual `linkMeshInfo` from the array output variable from `getMeshes` function.
%         - `w_H_link` : Homogeneous transform from the world to the link
%     - Outputs:
%         - `meshHandles`  : Struct that contains all the created handles for the meshes. There can be more than one mesh for each link.
%         - `transform`            : The transform object for the link
% 
% Author : Francisco Andrade (franciscojavier.andradechavez@iit.it)
%
% Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.

        
% Default values
meshColor= [0.1843    0.3098    0.3098];
transparency=0.5;

% create a transform object %
transform = hgtransform('Parent',gca);
for mesh_number=1:length(linkMeshInfo.meshInfo)
    
    mesh_triangles=linkMeshInfo.meshInfo(mesh_number).mesh_triangles;
    link_H_geom=linkMeshInfo.meshInfo(mesh_number).link_H_geom;
    scale=linkMeshInfo.meshInfo(mesh_number).scale;
    
    % Change scale
    % Scale the STL mesh    
    corrected_vertices=mesh_triangles.Points.*scale;    
    
    % Applying transform to link frame to stl file  :    
    corrected_vertices=link_H_geom*[corrected_vertices';ones(1,size(corrected_vertices,1))];
    corrected_vertices=corrected_vertices';
    corrected_vertices=corrected_vertices(:,1:3);
    
    % plot in figureusing patch
    modelMesh(mesh_number) = ...
        patch('Faces',mesh_triangles.ConnectivityList,'Vertices',corrected_vertices,...
        'FaceColor',meshColor , ...
        'EdgeColor',  'none',        ...
        'FaceLighting',    'gouraud',     ...
        'AmbientStrength', 0.25);
    
    %parent transform as parent to the mesh
    set(modelMesh(mesh_number),'Parent',transform);
    
    % Set the object transparency
    alpha(modelMesh(mesh_number),transparency);
    
end
set(transform,'Matrix',w_H_link);
meshHandles.modelMesh=modelMesh;
end
