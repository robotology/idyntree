function velCoM = getCenterOfMassVelocity(KinDynModel)

    % GETCENTEROFMASSVELOCITY retrieves the CoM velocity in world coordinates.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  velCoM = getCenterOfMassVelocity(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - velCoM: [3 x 1] CoM velocity w.r.t. world frame.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------  
    
    % get the CoM velocity
    velCoM_iDyntree = KinDynModel.kinDynComp.getCenterOfMassVelocity(); 
    
    % covert to matlab
    velCoM = velCoM_iDyntree.toMatlab;
end
