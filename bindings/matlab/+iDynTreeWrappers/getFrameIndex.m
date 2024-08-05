function frameID = getFrameIndex(KinDynModel,frameName)

    % GETFRAMEINDEX gets the index corresponding to a given frame name.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  frameID = getFrameIndex(KinDynModel,frameName)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %          - frameName: a string specifying a valid frame name.
    %
    % OUTPUTS: - frameID: the ID associated to the given frame name.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % get the ID of the given frame
    frameID = KinDynModel.kinDynComp.getFrameIndex(frameName);
end
