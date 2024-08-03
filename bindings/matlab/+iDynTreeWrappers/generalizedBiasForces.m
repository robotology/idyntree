function h = generalizedBiasForces(KinDynModel)

    % GENERALIZEDBIASFORCES retrieves the generalized bias forces from
    %                       the reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  h = generalizedBiasForces(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - h: [6+ndof x 1] generalized bias accelerations.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % get the bias forces
    ack = KinDynModel.kinDynComp.generalizedBiasForces(KinDynModel.dynamics.h_iDyntree);

    % check for errors
    if ~ack
        error('[generalizedBiasForces]: unable to get the bias forces from the reduced model.')
    end

    % convert to Matlab format: compute the base bias acc (h_b) and the
    % joint bias acc (h_s) and concatenate them
    h_b = KinDynModel.dynamics.h_iDyntree.baseWrench.toMatlab;
    h_s = KinDynModel.dynamics.h_iDyntree.jointTorques.toMatlab;
    h   = [h_b;h_s];
end
