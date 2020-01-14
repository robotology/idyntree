function basePose = getWorldBaseTransform(KinDynModel)

    % GETWORLDBASETRANSFORM gets the transformation matrix between the base frame
    %                            and the inertial reference frame. 
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  basePose = getWorldBaseTransform(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - basePose: [4 x 4] from base to world transformation matrix.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    % get the transformation between the base and the world frames 
    basePose_iDyntree     = KinDynModel.kinDynComp.getWorldBaseTransform();  
    baseRotation_iDyntree = basePose_iDyntree.getRotation;
    baseOrigin_iDyntree   = basePose_iDyntree.getPosition;
    
    % covert to Matlab format
    baseRotation          = baseRotation_iDyntree.toMatlab;
    baseOrigin            = baseOrigin_iDyntree.toMatlab;
    basePose              = [baseRotation, baseOrigin;
                                0,   0,   0,   1];                  
   % Debug output
    if KinDynModel.DEBUG
        
        disp('[getWorldBaseTransform]: debugging outputs...')
        
        % baseRotation = basePose(1:3,1:3) must be a valid rotation matrix
        if det(basePose(1:3,1:3)) < 0.9 || det(basePose(1:3,1:3)) > 1.1
            
            error('[getWorldBaseTransform]: baseRotation is not a valid rotation matrix.')
        end
        
        IdentityMatr = basePose(1:3,1:3)*basePose(1:3,1:3)';
        
        for kk = 1:size(IdentityMatr, 1)
            
            for jj = 1:size(IdentityMatr, 1)
                
                if jj == kk
                    
                    if abs(IdentityMatr(kk,jj)) < 0.9 || abs(IdentityMatr(kk,jj)) > 1.1
                        
                        error('[getWorldBaseTransform]: baseRotation is not a valid rotation matrix.')
                    end
                else
                    if abs(IdentityMatr(kk,jj)) > 0.01
                        
                        error('[getWorldBaseTransform]: baseRotation is not a valid rotation matrix.')
                    end
                end
            end   
        end       
             
        disp('[getWorldBaseTransform]: done.')     
    end   
end
