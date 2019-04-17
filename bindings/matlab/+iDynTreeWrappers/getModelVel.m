function stateVel = getModelVel(KinDynModel)

    % GETMODELVEL gets the joints and floating base velocities from the 
    %                  reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  stateVel = getModelVel(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - stateVel: [6+ndof x 1] vector of joints and base velocities.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    % create the vector that must be populated with the stae velocities
    stateVel_iDyntree = iDynTree.VectorDynSize(KinDynModel.NDOF);
    
    % get the joints velocities
    ack = KinDynModel.kinDynComp.getModelVel(stateVel_iDyntree);
    
    % check for errors
    if ~ack  
        error('[getModelVel]: unable to retrieve the state velocities from the reduced model.')
    end
    
    % convert to Matlab format
    stateVel = stateVel_iDyntree.toMatlab;
end
