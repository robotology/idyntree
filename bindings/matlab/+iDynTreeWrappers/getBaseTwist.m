function baseVel = getBaseTwist(KinDynModel)

    % GETBASETWIST retrieves the robot base velocity from the reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  baseVel = getBaseTwist(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - baseVel: [6 x 1] vector of base linear and angular velocity;
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------

    % get the base velocities
    baseVel_iDyntree = KinDynModel.kinDynComp.getBaseTwist();
    
    % convert to Matlab format
    baseVel = baseVel_iDyntree.toMatlab;
end
