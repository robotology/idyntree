function jointPos = getJointPos(KinDynModel)

    % GETJOINTPOS retrieves the joints configuration from the reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  jointPos = getJointPos(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - jointPos: [ndof x 1] vector of joint positions.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % get the joints positions
    ack = KinDynModel.kinDynComp.getJointPos(KinDynModel.kinematics.jointPos_iDyntree);

    % check for errors
    if ~ack
        error('[getJointPos]: unable to retrieve the joint positions from the reduced model.')
    end

    % convert to Matlab format
    jointPos = KinDynModel.kinematics.jointPos_iDyntree.toMatlab;
end
