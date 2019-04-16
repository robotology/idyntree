function totalMomentum = getCentroidalTotalMomentum(KinDynModel)

    % GETCENTROIDALTOTALMOMENTUM retrieves the centroidal momentum from the reduced
    %                                 model. The quantity is expressed in a frame that depends
    %                                 on the 'FrameVelocityReperesentation' settings.                               
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  totalMomentum = getCentroidalTotalMomentum(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - totalMomentum: [6 x 1] vector of linear and angular momentum.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    % get the momentum
    totalMomentum_iDyntree = KinDynModel.kinDynComp.getCentroidalTotalMomentum();
    
    % convert to Matlab format
    totalMomentum = totalMomentum_iDyntree.toMatlab;
end
