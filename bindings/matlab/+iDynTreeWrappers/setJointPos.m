function [] = setJointPos(KinDynModel,jointPos)

    % SETJOINTPOS sets the joints configuration for kino-dynamic 
    %                  computations.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT: [] = setJointPos(KinDynModel,jointPos)
    %
    % INPUTS: - jointPos: [ndof x 1] vector representing the joints 
    %                     configuration in radians;
    %         - KinDynModel: a structure containing the loaded model and additional info.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------

    % Debug input
    if KinDynModel.DEBUG
        
        disp('[setJointPos]: debugging inputs...')
        
        % check joints position vector size
        if length(jointPos) ~= KinDynModel.NDOF
            
            error('[setJointPos]: the length of jointPos is not KinDynModel.NDOF')
        end
            
        disp('[setJointPos]: done.')     
    end
    
    % convert the joint position to a dynamic size vector
    jointPos_iDyntree = iDynTree.VectorDynSize(KinDynModel.NDOF);
    
    for k = 0:length(jointPos)-1
        
        jointPos_iDyntree.setVal(k,jointPos(k+1));
    end
    
    % set the current joint positions
    ack = KinDynModel.kinDynComp.setJointPos(jointPos_iDyntree);
    
    % check for errors
    if ~ack  
        error('[setJointPos]: unable to set the joint positions.')
    end  
end
