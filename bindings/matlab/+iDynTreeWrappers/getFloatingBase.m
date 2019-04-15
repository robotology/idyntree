function baseLinkName = getFloatingBase(KinDynModel)

    % GETFLOATINGBASE retrieves the floating base link name from the 
    %                      reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  baseLinkName = getFloatingBase(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - baseLinkName: name of the base link.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

    %% ------------Initialization----------------
    
    % get the name of the floating base link
    baseLinkName = KinDynModel.kinDynComp.getFloatingBase();  
end
