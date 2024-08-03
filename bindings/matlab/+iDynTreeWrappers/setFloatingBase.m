function [] = setFloatingBase(KinDynModel,floatBaseLinkName)

    % SETFLOATINGBASE sets the link that is used as floating base.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT: [] = setFloatingBase(KinDynModel,floatBaseLinkName)
    %
    % INPUTS: - floatBaseLinkName: a string with the name of the link to be
    %                              used as floating base;
    %         - KinDynModel: a structure containing the loaded model and additional info.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % set the floating base link
    ack = KinDynModel.kinDynComp.setFloatingBase(floatBaseLinkName);

    % check for errors
    if ~ack
        error('[setFloatingBase]: unable to set the floating base link.')
    end
end
