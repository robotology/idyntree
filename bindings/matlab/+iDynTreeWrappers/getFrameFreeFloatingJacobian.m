function J_frame = getFrameFreeFloatingJacobian(KinDynModel,frameName)

    % GETFRAMEFREEFLOATINGJACOBIAN gets the free floating jacobian of a
    %                              specified frame.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  J_frame = getFrameFreeFloatingJacobian(KinDynModel,frameName)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %          - frameName: a string that specifies the frame w.r.t. compute the
    %                       jacobian matrix, or the associated ID;
    %
    % OUTPUTS: - J_frame: [6 x 6+ndof] frame free floating Jacobian.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % get the free floating jacobian
    ack = KinDynModel.kinDynComp.getFrameFreeFloatingJacobian(frameName,KinDynModel.kinematics.J_frame_iDyntree);

    % check for errors
    if ~ack
        error('[getFrameFreeFloatingJacobian]: unable to get the Jacobian from the reduced model.')
    end

    % convert to Matlab format
    J_frame = KinDynModel.kinematics.J_frame_iDyntree.toMatlab;
end
