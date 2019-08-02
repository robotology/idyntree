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
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    % set the floating base link
    ack = KinDynModel.kinDynComp.setFloatingBase(floatBaseLinkName);
    
    % check for errors
    if ~ack  
        error('[setFloatingBase]: unable to set the floating base link.')
    end  
end
