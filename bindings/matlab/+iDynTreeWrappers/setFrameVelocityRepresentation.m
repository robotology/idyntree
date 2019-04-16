function [] = setFrameVelocityRepresentation(KinDynModel,frameVelRepr)

    % SETFRAMEVELOCITYREPRESENTATION sets the frame velocity representation.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT: [] = setFrameVelocityRepresentation(KinDynModel,frameVelRepr)
    %
    % INPUTS:  - frameVelRepr: a string with one of the following values:
    %                          'mixed', 'body', 'inertial';
    %          - KinDynModel: a structure containing the loaded model and additional info.
    %
    % Mapping for the frame velocity reperesentation
    %
    %  0 = INERTIAL_FIXED_REPRESENTATION
    %
    %  1 = BODY_FIXED_REPRESENTATION
    %
    %  2 = MIXED_REPRESENTATION
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------

    switch frameVelRepr
        
        case 'mixed'
            
            frameVelRepr_idyntree = iDynTree.MIXED_REPRESENTATION;
                    
        case 'body'
            
            frameVelRepr_idyntree = iDynTree.BODY_FIXED_REPRESENTATION;
                    
        case 'inertial'
            
            frameVelRepr_idyntree = iDynTree.INERTIAL_FIXED_REPRESENTATION;
                    
        otherwise        
            error('[setFrameVelocityRepresentation]: frameVelRepr is not a valid string.')
    end
             
    % set the desired frameVelRepr
    ack = KinDynModel.kinDynComp.setFrameVelocityRepresentation(frameVelRepr_idyntree);
    
    % check for errors
    if ~ack    
        error('[setFrameVelocityRepresentation]: unable to set the frame velocity representation.')
    end
end
