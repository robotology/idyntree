function velCoM = getCenterOfMassVelocity(KinDynModel)

    % GETCENTEROFMASSVELOCITY retrieves the CoM velocity in world coordinates.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  velCoM = getCenterOfMassVelocity(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - velCoM: [3 x 1] CoM velocity w.r.t. world frame.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

    %% ------------Initialization----------------  
    
    % get the CoM velocity
    velCoM_iDyntree = KinDynModel.kinDynComp.getCenterOfMassVelocity(); 
    
    % covert to matlab
    velCoM = velCoM_iDyntree.toMatlab;
end
