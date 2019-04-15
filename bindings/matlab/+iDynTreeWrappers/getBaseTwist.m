function baseVel = getBaseTwist(KinDynModel)

    % GETBASETWIST retrieves the robot base velocity from the reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  baseVel = getBaseTwist(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - baseVel: [6 x 1] vector of base linear and angular velocity;
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

    %% ------------Initialization----------------

    % get the base velocities
    baseVel_iDyntree = KinDynModel.kinDynComp.getBaseTwist();
    
    % convert to Matlab format
    baseVel = baseVel_iDyntree.toMatlab;
end
