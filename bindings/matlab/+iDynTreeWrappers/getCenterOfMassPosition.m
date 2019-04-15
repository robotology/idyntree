function posCoM = getCenterOfMassPosition(KinDynModel)

    % GETCENTEROFMASSPOSITION retrieves the CoM position in world coordinates.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  posCoM = getCenterOfMassPosition(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - posCoM: [3 x 1] CoM position w.r.t. world frame.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

    %% ------------Initialization----------------
    
    % get the CoM position
    posCoM_iDyntree = KinDynModel.kinDynComp.getCenterOfMassPosition(); 
    
    % covert to matlab
    posCoM = posCoM_iDyntree.toMatlab;
end
