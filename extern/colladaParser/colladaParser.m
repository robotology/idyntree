%COLLADAPARSER parses a collada file for triangular elements. 
%main use: unroll scene graph and obtain a list of primitives on the scene
%that can be plotted. 
%SYNTAX: 
%     [sceneElements, sceneT] = COLLADAPARSER(colFile)
%   INPUT:
%     colFile - collada file read using xml2struct 
%               e.g., colFile = xml2struct(<file>); 
%   OUTPUT:
%     sceneElements - array of structures containing triangular elements 
%          Fields:
%            tri - contains a 3xNumTri list of vertices that form triangles 
%            vertex - contains a list vertices used (Nvertices x 3)
%     sceneT -  list of transformations applied to geometric instances
%
%NOTE: this file only reads triangular geometric elements. 
%All other primitive types will be ignored (and a warning will be issued)
%
%
% Randi Cabezas
%
% Last Edited: 2/25/14 (added up_axis parsing)
% Created: 2/10/14

function [sceneElements, sceneT] = colladaParser(colFile)

    %sceneT = struct([]); 
    sceneT = struct('T',[],'nodeID',[],'geometryID',[],'fillIndex',[],'allocated',[]);
    [sceneT(end+1:end+500).fillIndex] = deal([]); 
    sceneT(1).fillIndex = 0; 
    %this is hack to reduce the number of times the array of structures
    %needs to be expanded (which is very slow), instead allocate some bulk now, then continue to
    %allocate, just have to be careful on appending at the end of the last
    %valid append (indicated by the sceneT(1).fillIndex)

    
    %check the up axis (if not Z_UP, then a base transformation is needed to be z_up)
    upText = colFile.COLLADA.asset.up_axis.Text;
    if strcmp(upText,'Z_UP')
        T = eye(4); 
    elseif strcmp(upText,'Y_UP')
        T = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1]; 
    elseif strcmp(upText,'X_UP')
        T = [0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1];  %need to check if this is correct
    else
        error('Up axis not found'); 
    end

    
    
    nNode = length(colFile.COLLADA.library_visual_scenes.visual_scene.node); 
    if(nNode==1)
        colFile.COLLADA.library_visual_scenes.visual_scene.node = {colFile.COLLADA.library_visual_scenes.visual_scene.node}; 
    end
  
    
    %gets a list of nodes in the visual scene 
    for i=1:nNode
        curNode = colFile.COLLADA.library_visual_scenes.visual_scene.node{i}; 
        sceneT = nodeCollect(curNode, sceneT, T); 
    end

    
    
    
    %repeat for library_node with the exception that we now must search for
    %the nodeID on the sceneT structure and chain the transformations
    %properly 
    if isfield(colFile.COLLADA,'library_nodes') 
        nNode = length(colFile.COLLADA.library_nodes.node); 
        if(nNode==1) 
           colFile.COLLADA.library_nodes.node = {colFile.COLLADA.library_nodes.node}; 
        end

        for i=1:nNode
            curNode = colFile.COLLADA.library_nodes.node{i}; 
            id = curNode.Attributes.id; 

            %this needs to be done everytime to catch nested dependances in
            %library_node
            idToSearch={sceneT(:).nodeID};

            found = find(strcmp(id,idToSearch)); 

            if(isempty(found))
                prevT = eye(4); 
            else
                prevT = {sceneT(found).T};
            end


            %processes this node (and possible sub-nodes) with each of the
            %previous transformations 
            %this could be speed-up by processing in parallel (i.e. multiple
            %transformations at once) but I don't feel like further
            %complicating the nested function calls
            for j=1:length(found)
                sceneT = nodeCollect(curNode,sceneT,prevT{j});
            end
        end
    end
        
    sceneT(sceneT(1).fillIndex+1:end) =[];

    %now we have unrolled the scene graph and have a list of
    %transformations to apply to instanced geometry 
    %(instanced nodes should have been unrolled as well)
        
    sceneElements = collectGeometry(sceneT,colFile.COLLADA.library_geometries.geometry); 
    
    
    
end


function [sceneT] = nodeCollect(curNode,sceneT, prevT) 
%this function recusively traverses the node elements to fill sceneT until
%no more node children exist (properly chaining transformations). 
%While doing so it fills geometryID with instance_geometry and nodeID with
%instance_node (which is an intermediate bookeeping variable). 

    if(isfield(curNode,'matrix')) 
        %gets transformation matrix as specified by this node
        curT = reshape(str2num(curNode.matrix.Text),4,4)'; 
    else
        curT = eye(4); 
    end
    
    %chaining the transformations 
    T = prevT*curT; 
       
    %searches for instanced fields 
    sceneT = instancedGeometrySearch(curNode,sceneT,T); 
    sceneT = instancedNodeSearch(curNode,sceneT, T); 
    
    
    %recursive call if more node fields exist 
    if(isfield(curNode,'node'))
        nNode = length(curNode.node); 
        
        if(nNode==1)
            %dumb fix to avoid replicating code when we have structure
            %rather than cell array of structures
             curNode.node = {curNode.node}; 
        end
        
        for i=1:nNode
            sceneT = nodeCollect(curNode.node{i}, sceneT,T); 
        end
    end
    
    
end


function [sceneT] = instancedNodeSearch(curNode,sceneT, Transformation)
%this function searches for instanced_nodes and adds them to nodeID in
%sceneT (the assumption is that we will need to search for these nodes
%later in the xml document to unroll them into we get instanced_geometry)

    instanceFields = {'instance_node'}; 
    sceneTField = {'nodeID'};

    %early out if no instanceField
    if ~isfield(curNode,instanceFields) 
        return; 
    end
    
    %searches for instanced fields  
    nField = length(curNode.(instanceFields{1})); 

    if(nField==1) 
        %dumb wrapper to avoid replicating code when the input is structure rather than cell array of structures
        %this is an inconsistency from xml2struct
        curNode.(instanceFields{1}) = {curNode.(instanceFields{1})};    
    end

    base = sceneT(1).fillIndex;
    if(length(sceneT) <= (base + nField +1))
        [sceneT(end+1:end+2*length(sceneT)).T] = deal([]); 
    end
    
    
    sceneT(1).fillIndex = sceneT(1).fillIndex + nField; 
    
    for k=1:nField
        url = curNode.(instanceFields{1}){k}.Attributes.url; 
        sceneT(base+k).T = Transformation; 
        sceneT(base+k).(sceneTField{1}) =url(2:end); %gets rid of the reference (#)
    end
  
end

function [sceneT] = instancedGeometrySearch(curNode,sceneT, Transformation)
%this function searched for instanced geometry and add them to sceneT with
%the correct transformation needed for this geometry

    instanceFields = {'instance_geometry'}; 
    sceneTField = {'geometryID'};

     %early out if no instanceField
    if ~isfield(curNode,instanceFields) 
        return; 
    end
    
    %searches for instanced fields  
    nField = length(curNode.(instanceFields{1})); 

    if(nField==1) 
        %dumb wrapper to avoid replicating code when the input is structure rather than cell array of structures
        %this is an inconsistency from xml2struct
        curNode.(instanceFields{1}) = {curNode.(instanceFields{1})};    
    end

    
    
    base = sceneT(1).fillIndex;
    if(length(sceneT) <= (base + nField +1))
        [sceneT(end+1:end+2*length(sceneT)).T] = deal([]); 
    end
    
    sceneT(1).fillIndex = sceneT(1).fillIndex + nField; 
    
    for k=1:nField
        url = curNode.(instanceFields{1}){k}.Attributes.url; 
        sceneT(base+k).T = Transformation; 
        sceneT(base+k).(sceneTField{1}) =url(2:end); %gets rid of the reference (#)
    end


end




function sceneElements = collectGeometry(sceneT,geometry) 


%this is a hack to reduce the size of the strcmp as we loop through the
%geometry
geometryIDToSearch={sceneT(:).geometryID};
idx = 1:length(sceneT); 



sources = struct([]); 
semantic = struct([]); 
triangles = struct([]); 

sceneElements =struct([]); 



nGeo = length(geometry); 
if(nGeo==1)
    geometry = {geometry}; 
end



for i=1:nGeo

    curGeo = geometry{i};
    
    if(~isfield(curGeo,'mesh'))
       warning('No mesh children found on geometry (%d)...skipping.',i);  
       continue; 
    end
    
    
    if(~isfield(curGeo.mesh,'triangles'))
       warning('No triangle children found on geometry (%d)...skipping it.',i);
       continue; 
    end
    
  
    
    %collect sources, vertices and triangles
    sources = collectSource(curGeo.mesh.source, sources); 
    semantic = collectVertices(curGeo.mesh.vertices, semantic);  
    
    %gather the necessary information for this triangle[s]
    triangles = collectTriangles(curGeo.mesh.triangles, triangles, sources, semantic); 

    %need to back out which indecies where added to triangles 
    nTri = length(curGeo.mesh.triangles); 
    index = length(triangles)-nTri+1:length(triangles); 
    
    id = curGeo.Attributes.id;
    
    %get a list of transformations to apply to current primitive
    found = find(strcmp(id,geometryIDToSearch)); 
    
        
    if(isempty(found))
       T = {eye(4)}; 
    else
        
        T = {sceneT(idx(found)).T}; %scene graph transformation 
        
        %delete from the search vector
        %geometryIDToSearch(found)=[];
        [geometryIDToSearch{found}]=deal([]);
        %idx(found) = []; 
    end
    
    for j=1:length(T)
        for k=index

            %transformed vertices 
            tVertex = T{j}*[triangles(k).vertices'; ones(1,size(triangles(k).vertices,1))];

            tVertex = tVertex'; 

            %add them to the stack
            sceneElements(end+1).tri = triangles(k).triIndex; 
            sceneElements(end).vertex = tVertex(:,1:3); 
        end
    end
    

end


end


function sources = collectSource(curSources, sources) 
%populates the sources structure with fields id, and data (raw)

    %possible data types
    sourceField = {'IDREF_array','Name_array','bool_array','float_array','int_array'}; 
    
           
    %searches for instanced fields  
    nSource = length(curSources); 

    if(nSource==1) 
        %dumb wrapper to avoid replicating code when the input is structure rather than cell array of structures
        %this is an inconsistency from xml2struct
        curSources = {curSources};    
    end

    for k=1:nSource
                
        sources(end+1).id = curSources{k}.Attributes.id; 
    
        idx = isfield(curSources{k},sourceField);
        
        if(isempty(idx)) %skips empty sources 
            continue; 
        end
        
        sources(end).dat = str2num(curSources{k}.(sourceField{idx}).Text); 

    end
  


end


function semantic= collectVertices(curVertices, semantic) 
%fields: id, posSourceID, other


    [pos, other] = collectInput(curVertices.input,'position');

    semantic(end+1).id = curVertices.Attributes.id; 
    semantic(end).posSourceID = pos.source;    
    semantic(end).other = other; 
   

end



function triangles = collectTriangles(curTri, triangles, sources, semantic)
%collects the triangle information 
% fields: vertices, triIndex, vertexID and p

    nTri = length(curTri); 
    if(nTri==1)
        curTri = {curTri}; 
    end
    
    semanticIDToSearch = {semantic.id}; 
    sourcesIDToSearch = {sources.id}; 
    
    for i=1:nTri
        %note that this vertex is not really a vertex, but rather the
        %semantic meaning that points to the vertex data. 
        [vertex, other]= collectInput(curTri{i}.input,'vertex'); 

        triangles(end+1).vertexID = vertex.source; 

        %grabs raw primitive info (still need to break it up by the
        %corresponding offsets)
        p = []; 
        if(isfield(curTri{i},'p'))
            p = str2num(curTri{i}.p.Text);    

            %typically the vertex offset is 0 since it is required..but just being
            %careful
            %skip is more compicated it could be no skip (1) or skip one (2) 
            %depending on what else is provided (i..e texcoord). 
            start=1+vertex.offset; 
            skip =length(vertex)+length(other);  

            %gets rid of the coordiante offsets in the primitives
            triIndex = reshape(p(start:skip:end),3,[])+1;

            triangles(end).triIndex = triIndex; 
        end

        triangles(end).p = p; 
        
        % looks up the semantic index in semantic structure
        semanticLoc = strcmp(triangles(end).vertexID,semanticIDToSearch); 
        
        %now looks up the source given by semantic in the source structure
        sourceLoc = strcmp(semantic(semanticLoc).posSourceID,sourcesIDToSearch); 
        
        %storing the source for good measure
        triangles(end).sourceID = semantic(semanticLoc).posSourceID; 
        
        %getting the raw (source) data and building the list of triangle vertices
        triangles(end).vertices = reshape(sources(sourceLoc).dat,3,[])';
    
    end
end




function [outI, other] = collectInput(input,semanticLabelInterest)
%collects inputs, returns the semantic label of interest in the first
%output all other ones in the second output (label of interest could be
%empty then all output will be in other)
%fields: source, semantic, and offset [if exists]


    out = struct([]); 
    idx = []; 

    nInput = length(input); 
    if(nInput==1)
       input = {input};
    end
    
    for i=1:nInput
       out(end+1).source = input{i}.Attributes.source(2:end); %gets rid of #

       
       if(isfield(input{i}.Attributes,'offset'))
           out(end).offset = str2double(input{i}.Attributes.offset);
       end
       
       
       out(end).semantic = lower(input{i}.Attributes.semantic); 

       if strcmp(out(end).semantic ,semanticLabelInterest)
           idx = i; 
       end
       
    end
    
    other = out; 
    
    if ~isempty(idx)
        outI = out(idx); 
        other(idx) = []; 
    end
        

end









