function g = generalizedGravityForces(KinDynModel)

    % GENERALIZEDGRAVITYFORCES retrieves the generalized gravity forces 
    %                               given the reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  g = generalizedGravityForces(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - g: [6+ndof x 1] generalized gravity forces.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

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
