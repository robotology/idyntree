function stateVel = getModelVel(KinDynModel)

    % GETMODELVEL gets the joints and floating base velocities from the
    %             reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  stateVel = getModelVel(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - stateVel: [6+ndof x 1] vector of joints and base velocities.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % get the joints velocities
    ack = KinDynModel.kinDynComp.getModelVel(KinDynModel.kinematics.stateVel_iDyntree);

    % check for errors
    if ~ack
        error('[getModelVel]: unable to retrieve the state velocities from the reduced model.')
    end

    % convert to Matlab format
    stateVel = KinDynModel.kinematics.stateVel_iDyntree.toMatlab;
end
