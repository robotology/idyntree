function [] = setJointPos(KinDynModel,jointPos)

    % SETJOINTPOS sets the joints configuration for kino-dynamic computations.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT: [] = setJointPos(KinDynModel,jointPos)
    %
    % INPUTS: - jointPos: [ndof x 1] vector representing the joints
    %                     configuration in radians;
    %         - KinDynModel: a structure containing the loaded model and additional info.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % Debug input
    if KinDynModel.DEBUG

        disp('[setJointPos]: debugging inputs...')

        % check joints position vector size
        if length(jointPos) ~= KinDynModel.NDOF

            error('[setJointPos]: the length of jointPos is not KinDynModel.NDOF')
        end

        disp('[setJointPos]: done.')
    end

    for k = 0:length(jointPos)-1

        KinDynModel.kinematics.jointPos_iDyntree.setVal(k,jointPos(k+1));
    end

    % set the current joint positions
    ack = KinDynModel.kinDynComp.setJointPos(KinDynModel.kinematics.jointPos_iDyntree);

    % check for errors
    if ~ack
        error('[setJointPos]: unable to set the joint positions.')
    end
end
