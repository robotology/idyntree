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
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    % get the ID of the given frame
    frameID = KinDynModel.kinDynComp.getFrameIndex(frameName); 
end
