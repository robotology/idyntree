function frameName = getFrameName(KinDynModel,frameID)

    % GETFRAMENAME gets the name corresponding to a given frame index.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  frameName = getFrameName(KinDynModel,frameID)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %          - frameID: the ID associated to the given frame name.
    %
    % OUTPUTS: - frameName: a string specifying a valid frame name.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------

    % get the ID of the given frame
    frameName = KinDynModel.kinDynComp.getFrameName(frameID);   
end
