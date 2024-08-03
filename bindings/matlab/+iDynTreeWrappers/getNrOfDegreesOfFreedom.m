function nDof = getNrOfDegreesOfFreedom(KinDynModel)

    % GETNROFDEGREESOFFREEDOM gets the dimension of the joint space.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  nDof = getNrOfDegreesOfFreedom(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - nDof: number of DoFs of the system.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % get the number of DoF
    nDof = KinDynModel.kinDynComp.getNrOfDegreesOfFreedom();

    % Debug output
    if KinDynModel.DEBUG

        disp('[getNrOfDegreesOfFreedom]: debugging outputs...')

        % check nDof is not empty
        if isempty(nDof)

            warning('[getNrOfDegreesOfFreedom]: nDof is empty.')
        end

        disp('[getNrOfDegreesOfFreedom]: done.')
    end
end
