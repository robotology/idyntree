function h = generalizedBiasForces(KinDynModel)

    % GENERALIZEDBIASFORCES retrieves the generalized bias forces from 
    %                            the reduced model. 
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  h = generalizedBiasForces(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - h: [6+ndof x 1] generalized bias accelerations.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    % create the vector that must be populated with the bias forces
    h_iDyntree = iDynTree.FreeFloatingGeneralizedTorques(KinDynModel.kinDynComp.model);
    
    % get the bias forces
    ack = KinDynModel.kinDynComp.generalizedBiasForces(h_iDyntree);
    
    % check for errors
    if ~ack
        error('[generalizedBiasForces]: unable to get the bias forces from the reduced model.')
    end
    
    % convert to Matlab format: compute the base bias acc (h_b) and the
    % joint bias acc (h_s) and concatenate them
    h_b = h_iDyntree.baseWrench.toMatlab;
    h_s = h_iDyntree.jointTorques.toMatlab;   
    h   = [h_b;h_s];
end
