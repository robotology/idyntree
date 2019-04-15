function frameVelRepr = getFrameVelocityRepresentation(KinDynModel)

    % GETFRAMEVELOCITYREPRESENTATION retrieves the current frame velocity
    %                                     representation. 
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  frameVelRepr = getFrameVelocityRepresentation(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - frameVelRepr: a string with one of the following possible values:
    %                          'mixed', 'body', 'inertial';
    %
    % Possible frame velocity representations:
    %
    %  0 = INERTIAL_FIXED_REPRESENTATION 	
    %
    %  1 = BODY_FIXED_REPRESENTATION
    %
    %  2 = MIXED_REPRESENTATION
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

    %% ------------Initialization----------------
    
    frameVelRepr_idyntree = KinDynModel.kinDynComp.getFrameVelocityRepresentation();   
    
    % output the current frame velocity representation
    switch frameVelRepr_idyntree  
        
        case 2
            
            frameVelRepr = 'mixed';
                    
        case 1
            
            frameVelRepr = 'body';
                    
        case 0
            
            frameVelRepr = 'inertial';
                    
        otherwise        
            error('[setFrameVelocityRepresentation]: frameVelRepr is not a valid string.')
    end   
end
