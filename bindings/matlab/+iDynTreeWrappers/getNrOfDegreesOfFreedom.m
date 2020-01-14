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
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------

    % get the number of DoF
    nDof = KinDynModel.kinDynComp.getNrOfDegreesOfFreedom(); 
    
    % Debug output
    if KinDynModel.DEBUG
        
        disp('[getNrOfDegreesOfFreedom]: debugging outputs...')
        
        % check nDof is not empty
        if isempty(nDof)
            
            error('[getNrOfDegreesOfFreedom]: nDof is empty.')
        end
               
        disp('[getNrOfDegreesOfFreedom]: done.')     
    end
end
