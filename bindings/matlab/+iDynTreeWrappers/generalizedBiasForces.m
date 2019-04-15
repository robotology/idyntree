function h = generalizedBiasForces(KinDynModel)

    % GENERALIZEDBIASFORCES retrieves the generalized bias forces from 
    %                            the reduced model. 
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  h = generalizedBiasForces(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - h: [ndof+6 x 1] generalized bias accelerations.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

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
