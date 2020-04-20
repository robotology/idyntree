function w_H_frame = getWorldTransform(KinDynModel,frameName)

    % GETWORLDTRANSFORM gets the transformation matrix between a specified 
    %                        frame and the inertial reference frame.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  w_H_frame = getWorldTransform(KinDynModel,frameName)
    %
    % INPUTS:  - frameName: a string that specifies the frame w.r.t. compute the 
    %                       transfomation matrix, or the associated ID;     
    %          - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - w_H_frame: [4 x 4] from frame to world transformation matrix.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it), Francisco Andrade
    % (franciscojavier.andradechavez@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    % get the transformation between the frame and the world in Matlab
    % format
    w_H_frame = KinDynModel.kinDynComp.getWorldTransform(frameName).asHomogeneousTransform.toMatlab;  

    % Debug output
    if KinDynModel.DEBUG
        w_R_frame          = w_H_frame(1:3,1:3);
    
        disp('[getWorldTransform]: debugging outputs...')
        
         % w_R_frame must be a valid rotation matrix
         if det(w_R_frame) < 0.9 || det(w_R_frame) > 1.1
            
             error('[getWorldTransform]: w_R_frame is not a valid rotation matrix.')
         end
        
         IdentityMatr = w_H_frame(1:3,1:3)*w_H_frame(1:3,1:3)';
        
         for kk = 1:size(IdentityMatr, 1)
            
             for jj = 1:size(IdentityMatr, 1)
                
                 if jj == kk
                    
                     if abs(IdentityMatr(kk,jj)-1) > 0.9
                        
                         error('[getWorldTransform]: w_R_frame is not a valid rotation matrix.')
                     end
                 else
                     if abs(IdentityMatr(kk,jj)) > 0.1
                        
                          error('[getWorldTransform]: w_R_frame is not a valid rotation matrix.')
                     end
                 end
             end   
         end                          
         disp('[getWorldTransform]: done.')     
    end
end
