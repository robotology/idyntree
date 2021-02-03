function [linkMeshInfo,map]=getMeshes(model,meshFilePrefix)
% We use the iDyntree information to obtain the files containing the meshes
% and we link them to their link names
% Gets the mesh information for each link in the model.
%     - Iputs:
%         - `model` : iDyntree model loaded from a URDF.
%         - `meshFilePrefix` : Path in which we can find the meshes. As an example the path to the mesh in a iCub urdf is `'package://iCub/meshes/simmechanics/sim_sea_2-5_root_link_prt-binary.stl'
% `. `meshFilePrefix` should replace package to allow to find the rest of the path. If the value is "", the standard iDynTree workflow of locating the mesh via the ExternalMesh.getFileLocationOnLocalFileSystem method is used.
%     - Outputs:
%         - `map`        : Cell array having both the names of the meshes and the associated link
%         - `linkMeshInfo` : Struct array that contain the link name and a struct (`meshInfo`) that contains the name of file or if is a simple geometry, the triangulation ( edges and vertices of the mesh ) and the link to geometry transform.
%
% NOTE: at the moment only STL files are supported.
%
% Author : Francisco Andrade (franciscojavier.andradechavez@iit.it)
%
% Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.



% get the linkSolidShapes containing the mesh information
visual=model.visualSolidShapes;
linkSolidShapesV=visual.getLinkSolidShapes;
% get number of links
numberOfLinks=linkSolidShapesV.size;
linkMeshInfo=struct('meshInfo',{},'linkName',{});
% get pointer to beggining of vector
iterator=linkSolidShapesV.begin;
% iterate getting the name of the mesh and the name of the link
count=1;
link_with_no_visual=[];
for links=1:numberOfLinks
    linkName=model.getLinkName(links-1);
    linkMeshInfo(links).linkName=linkName;
    solidarray=iterator.next;
    solids_number=size(solidarray,2);
    meshInfo=struct('meshFile',{},'mesh_triangles',{},'link_H_geom',{});
    for solids=1:solids_number
        if solidarray{solids}.isExternalMesh
            externalMesh=solidarray{solids}.asExternalMesh;
            scale=externalMesh.getScale.toMatlab;
            if(meshFilePrefix == "")
               meshFile = externalMesh.getFilename;
               mesh_triangles = stlread(externalMesh.getFileLocationOnLocalFileSystem);
            else
                meshName=split(externalMesh.getFilename,':');
                meshFile=meshName{2};
                % Import an STL mesh, returning a PATCH-compatible face-vertex structure
                if strcmp('package',meshName{1})
                    mesh_triangles = stlread([meshFilePrefix meshFile]);
                else
                    mesh_triangles = stlread(meshFile);
                end
            end
            meshInfo(solids).meshFile=meshFile;
            meshInfo(solids).scale=scale';
        else
            meshInfo(solids).scale=[1,1,1];
            if solidarray{solids}.isCylinder
                meshInfo(solids).meshFile='cylinder';
                length=solidarray{solids}.asCylinder.getLength;
                radius=solidarray{solids}.asCylinder.getRadius;
                mesh_triangles=calculateMeshFromCylinder(length,radius);
            end
            if solidarray{solids}.isBox
                meshInfo(solids).meshFile='box';
                box_dimensions(1)=solidarray{solids}.asBox.getX;
                box_dimensions(2)=solidarray{solids}.asBox.getY;
                box_dimensions(3)=solidarray{solids}.asBox.getZ;
                mesh_triangles=calculateMeshFromBox(box_dimensions);
            end
            if solidarray{solids}.isSphere
                meshInfo(solids).meshFile='sphere';
                radius=solidarray{solids}.asSphere.getRadius;
                mesh_triangles=calculateMeshFromSphere(radius);
            end
        end
        link_H_geom=solidarray{solids}.getLink_H_geometry.asHomogeneousTransform.toMatlab;
        meshInfo(solids).link_H_geom=link_H_geom;
        meshInfo(solids).mesh_triangles=mesh_triangles;
        map(count,:)=[{meshInfo(solids).meshFile},{linkName}];
        count=count+1;
    end
    linkMeshInfo(links).meshInfo=meshInfo;
    if solids_number==0
       link_with_no_visual=[ link_with_no_visual links];
    end
end
% clean links with no visuals
linkMeshInfo(link_with_no_visual)=[];

end
%% Create points and connectivity list for each geometry

function [mesh_triangles]=calculateMeshFromSphere(radius)
[X,Y,Z]=sphere;
X = X * radius;
Y = Y * radius;
Z = Z * radius;
[F,V]=mesh2tri(X,Y,Z,'f');
mesh_triangles.Points=V;
mesh_triangles.ConnectivityList = F;
end

function [mesh_triangles]=calculateMeshFromBox(box_dimensions)
mesh_triangles.Points = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1];
mesh_triangles.Points(:,1)=mesh_triangles.Points(:,1)*box_dimensions(1)-box_dimensions(1)/2;
mesh_triangles.Points(:,2)=mesh_triangles.Points(:,2)*box_dimensions(2)-box_dimensions(2)/2;
mesh_triangles.Points(:,3)=mesh_triangles.Points(:,3)*box_dimensions(3)-box_dimensions(3)/2;
mesh_triangles.ConnectivityList = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
end

function [mesh_triangles]=calculateMeshFromCylinder(length,radius)
[X,Y,Z] = cylinder;
X = X * radius;
Y = Y * radius;
Z = Z * length-length/2;
[F,V]=mesh2tri(X,Y,Z,'f');
mesh_triangles.Points=V;
mesh_triangles.ConnectivityList = F;
end
