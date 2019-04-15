function stateVel = getModelVel(KinDynModel)

    % GETMODELVEL gets the joints and floating base velocities from the 
    %                  reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  stateVel = getModelVel(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - stateVel: [ndof+6 x 1] vector of joints and base velocities.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

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
