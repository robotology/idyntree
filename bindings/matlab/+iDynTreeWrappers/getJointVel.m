function jointVel = getJointVel(KinDynModel)

    % GETJOINTVEL retrieves joint velocities from the reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  jointVel = getJointVel(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - jointVel: [ndof x 1] vector of joint velocities.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

    %% ------------Initialization----------------

    % create the vector that must be populated with the joint velocities
    jointVel_iDyntree = iDynTree.VectorDynSize(KinDynModel.NDOF);
    
    % get the joints velocities
    ack = KinDynModel.kinDynComp.getJointVel(jointVel_iDyntree);
    
    % check for errors
    if ~ack   
        error('[getJointVel]: unable to retrieve the joint velocities from the reduced model.')
    end
    
    % convert to Matlab format
    jointVel = jointVel_iDyntree.toMatlab;
end
