function jointPos = getJointPos(KinDynModel)

    % GETJOINTPOS retrieves the joints configuration from the reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  jointPos = getJointPos(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - jointPos: [ndof x 1] vector of joint positions.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    % create the vector that must be populated with the joint positions
    jointPos_iDyntree = iDynTree.VectorDynSize(KinDynModel.NDOF);
    
    % get the joints positions
    ack = KinDynModel.kinDynComp.getJointPos(jointPos_iDyntree);
    
    % check for errors
    if ~ack   
        error('[getJointPos]: unable to retrieve the joint positions from the reduced model.')
    end
    
    % convert to Matlab format
    jointPos = jointPos_iDyntree.toMatlab;
end
