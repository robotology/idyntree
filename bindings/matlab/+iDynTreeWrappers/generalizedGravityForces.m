function g = generalizedGravityForces(KinDynModel)

    % GENERALIZEDGRAVITYFORCES retrieves the generalized gravity forces
    %                          given the reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  g = generalizedGravityForces(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - g: [6+ndof x 1] generalized gravity forces.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % get the gravity forces
    ack = KinDynModel.kinDynComp.generalizedGravityForces(KinDynModel.dynamics.g_iDyntree);

    % check for errors
    if ~ack
        error('[generalizedGravityForces]: unable to get the gravity forces from the reduced model.')
    end

    % convert to Matlab format: compute the base gravity forces (g_b) and the
    % joint gravity forces (g_j) and concatenate them
    g_b = KinDynModel.dynamics.g_iDyntree.baseWrench.toMatlab;
    g_s = KinDynModel.dynamics.g_iDyntree.jointTorques.toMatlab;
    g   = [g_b; g_s];
end
