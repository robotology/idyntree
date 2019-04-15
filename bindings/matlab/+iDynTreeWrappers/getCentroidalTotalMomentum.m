function totalMomentum = getCentroidalTotalMomentum(KinDynModel)

    % GETCENTROIDALTOTALMOMENTUM retrieves the centroidal momentum from the reduced
    %                                 model. The quantity is expressed in a frame that depends
    %                                 on the 'FrameVelocityReperesentation' settings.                               
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  totalMomentum = getCentroidalTotalMomentum(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - totalMomentum: [6 x 1] vector of linear and angular momentum.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

    %% ------------Initialization----------------
    
    % get the momentum
    totalMomentum_iDyntree = KinDynModel.kinDynComp.getCentroidalTotalMomentum();
    
    % convert to Matlab format
    totalMomentum = totalMomentum_iDyntree.toMatlab;
end
