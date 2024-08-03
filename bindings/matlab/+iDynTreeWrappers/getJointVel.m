function jointVel = getJointVel(KinDynModel)

    % GETJOINTVEL retrieves joint velocities from the reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  jointVel = getJointVel(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - jointVel: [ndof x 1] vector of joint velocities.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % get the joints velocities
    ack = KinDynModel.kinDynComp.getJointVel(KinDynModel.kinematics.jointVel_iDyntree);

    % check for errors
    if ~ack
        error('[getJointVel]: unable to retrieve the joint velocities from the reduced model.')
    end

    % convert to Matlab format
    jointVel = KinDynModel.kinematics.jointVel_iDyntree.toMatlab;
end
