function baseLinkName = getFloatingBase(KinDynModel)

    % GETFLOATINGBASE retrieves the floating base link name from the 
    %                      reduced model.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  baseLinkName = getFloatingBase(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - baseLinkName: name of the base link.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    % get the name of the floating base link
    baseLinkName = KinDynModel.kinDynComp.getFloatingBase();  
end
