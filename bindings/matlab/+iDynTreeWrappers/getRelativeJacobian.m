function J_frameVel = getRelativeJacobian(KinDynModel,frameVelID,frameRefID)

    % GETRELATIVEJACOBIAN gets the relative jacobian, i.e. the matrix that
    %                     maps the velocity of frameVel expressed w.r.t.
    %                     frameRef, to the joint velocity.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  J_frameVel = getRelativeJacobian(KinDynModel,frameVelID,frameRefID)
    %
    % INPUTS:  - frameRefID: a number that specifies the frame w.r.t. the velocity
    %                        of frameVel is expressed;
    %          - frameVelID: a number that specifies the frame whose velocity
    %                        is the one mapped by the jacobian;
    %          - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - J_frameVel: [6 x ndof] frameVel Jacobian.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % get the relative jacobian
    ack = KinDynModel.kinDynComp.getRelativeJacobian(frameVelID,frameRefID,KinDynModel.kinematics.J_frameVel_iDyntree);

    % check for errors
    if ~ack
        error('[getRelativeJacobian]: unable to get the relative jacobian from the reduced model.')
    end

    % covert to Matlab format
    J_frameVel = KinDynModel.kinematics.J_frameVel_iDyntree.toMatlab;
end
