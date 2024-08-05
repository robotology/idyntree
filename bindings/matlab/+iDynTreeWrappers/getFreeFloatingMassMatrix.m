function M = getFreeFloatingMassMatrix(KinDynModel)

    % GETFREEFLOATINGMASSMATRIX retrieves the free floating mass matrix.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  M = getFreeFloatingMassMatrix(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - M: [6+ndof x 6+ndof] free floating mass matrix.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % get the mass matrix
    ack = KinDynModel.kinDynComp.getFreeFloatingMassMatrix(KinDynModel.dynamics.M_iDyntree);

    % check for errors
    if ~ack
        error('[getFreeFloatingMassMatrix]: unable to retrieve the mass matrix from the reduced model.')
    end

    % convert to Matlab format
    M = KinDynModel.dynamics.M_iDyntree.toMatlab;

    % debug output
    if KinDynModel.DEBUG

        disp('[getFreeFloatingMassMatrix]: debugging outputs...')

        % check mass matrix symmetry and positive definiteness
        M_symm_error = norm(M -(M + M')/2);
        M_symm_eig   = eig((M + M')/2);

        if M_symm_error > 0.1

            error('[getFreeFloatingMassMatrix]: M is not symmetric')
        end

        for k = 1:length(M_symm_eig)

            if M_symm_eig(k) < 0

                error('[getFreeFloatingMassMatrix]: M is not positive definite')
            end
        end
        disp('[getFreeFloatingMassMatrix]: done.')
    end
end
