function g = generalizedGravityForces(KinDynModel)

    % GENERALIZEDGRAVITYFORCES retrieves the generalized gravity forces 
    %                               given the reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  g = generalizedGravityForces(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - g: [nDof+6 x 1] generalized gravity forces.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

    %% ------------Initialization----------------
    
    % create the vector that must be populated with the gravity forces
    g_iDyntree = iDynTree.FreeFloatingGeneralizedTorques(KinDynModel.kinDynComp.model);
    
    % get the gravity forces
    ack = KinDynModel.kinDynComp.generalizedGravityForces(g_iDyntree);
    
    % check for errors
    if ~ack  
        error('[generalizedGravityForces]: unable to get the gravity forces from the reduced model.')
    end
    
    % convert to Matlab format: compute the base gravity forces (g_b) and the
    % joint gravity forces (g_j) and concatenate them
    g_b = g_iDyntree.baseWrench.toMatlab;
    g_s = g_iDyntree.jointTorques.toMatlab;   
    g   = [g_b; g_s];
end
