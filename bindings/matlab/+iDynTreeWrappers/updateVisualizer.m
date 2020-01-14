function [] = updateVisualizer(Visualizer,KinDynModel,jointPos,basePose)

    % UPDATEVISUALIZER updates the iDyntree visualizer with the current
    %                  base pose and joints position.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % REQUIREMENTS: compile iDyntree with Irrlicht (IDYNTREE_USES_IRRLICHT = ON).
    %
    % FORMAT: [] = updateVisualizer(Visualizer,KinDynModel,jointPos,basePose)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %          - Visualizer: a structure containing the visualizer and its options.
    %          - jointPos: [ndof x 1] vector representing the joints
    %                      configuration in radians;
    %          - basePose: [4 x 4] from base frame to world frame transform.
    % 
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------

    % Debug input
    if Visualizer.DEBUG
        
        disp('[updateVisualizer]: debugging inputs...')
        
        % basePose must be a valid transformation matrix
        if size(basePose,1) ~= 4 || size(basePose,2) ~= 4
            
            error('[updateVisualizer]: basePose is not a 4x4 matrix.')
        end
        
        for ii = 1:4
                    
            if ii < 4
                        
                if abs(basePose(4,ii)) > 0.0001 
            
                    error('[updateVisualizer]: the last line of basePose is not [0,0,0,1].')
                end
            else
                if abs(basePose(4,ii)) > 1.0001 || abs(basePose(4,ii)) < 0.9999
            
                    error('[updateVisualizer]: the last line of basePose is not [0,0,0,1].')
                end
            end
        end
        
        % baseRotation = basePose(1:3,1:3) must be a valid rotation matrix
        if det(basePose(1:3,1:3)) < 0.9 || det(basePose(1:3,1:3)) > 1.1
            
            error('[updateVisualizer]: baseRotation is not a valid rotation matrix.')
        end
        
        IdentityMatr = basePose(1:3,1:3)*basePose(1:3,1:3)';
        
        for kk = 1:size(IdentityMatr, 1)
            
            for jj = 1:size(IdentityMatr, 1)
                
                if jj == kk
                    
                    if abs(IdentityMatr(kk,jj)) < 0.9 || abs(IdentityMatr(kk,jj)) > 1.1
                        
                        error('[updateVisualizer]: baseRotation is not a valid rotation matrix.')
                    end
                else
                    if abs(IdentityMatr(kk,jj)) > 0.01
                        
                        error('[updateVisualizer]: baseRotation is not a valid rotation matrix.')
                    end
                end
            end   
        end   
        
        % check joint position vector size
        if length(jointPos) ~= KinDynModel.NDOF
            
            error('[updateVisualizer]: the length of jointPos is not KinDynModel.NDOF')
        end
            
        disp('[updateVisualizer]: done.')     
    end
    
    % convert joints position to a dynamic size vector
    jointPos_iDyntree = iDynTree.VectorDynSize(KinDynModel.NDOF);
    
    for k = 0:length(jointPos)-1
        
        jointPos_iDyntree.setVal(k,jointPos(k+1));
    end
    
    % define the quantities required to set the floating base pose
    baseRotation_iDyntree = iDynTree.Rotation();
    baseOrigin_iDyntree   = iDynTree.Position();
    basePose_iDyntree     = iDynTree.Transform();
            
    % set the elements of the rotation matrix and of the base position vector
    for k = 0:2
                
        baseOrigin_iDyntree.setVal(k,basePose(k+1,4));
                
        for j = 0:2
                    
            baseRotation_iDyntree.setVal(k,j,basePose(k+1,j+1));                   
        end
    end
            
    % add the rotation matrix and the position to w_H_b_iDyntree
    basePose_iDyntree.setRotation(baseRotation_iDyntree);
    basePose_iDyntree.setPosition(baseOrigin_iDyntree);

    % set the current joints position and world-to-base transform
    ack = Visualizer.viz.modelViz(0).setPositions(basePose_iDyntree,jointPos_iDyntree);
       
    % check for errors
    if ~ack    
        error('[updateVisualizer]: unable to update the visualizer.')
    end
    
    Visualizer.viz.draw();
end
