function frameName = getFrameName(KinDynModel,frameID)

    % GETFRAMENAME gets the name corresponding to a given frame index.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  frameName = getFrameName(KinDynModel,frameID)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %          - frameID: the ID associated to the given frame name.
    %
    % OUTPUTS: - frameName: a string specifying a valid frame name.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

    %% ------------Initialization----------------

    % get the ID of the given frame
    frameName = KinDynModel.kinDynComp.getFrameName(frameID);   
end
