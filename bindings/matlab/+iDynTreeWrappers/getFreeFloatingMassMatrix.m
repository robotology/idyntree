function M = getFreeFloatingMassMatrix(KinDynModel)

    % GETFREEFLOATINGMASSMATRIX retrieves the free floating mass matrix.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  M = getFreeFloatingMassMatrix(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - M: [6+ndof x 6+ndof] free floating mass matrix.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    % create the matrix that must be populated with the mass info
    M_iDyntree = iDynTree.MatrixDynSize(KinDynModel.NDOF+6,KinDynModel.NDOF+6);
    
    % get the mass matrix
    ack = KinDynModel.kinDynComp.getFreeFloatingMassMatrix(M_iDyntree);
    
    % check for errors
    if ~ack  
        error('[getFreeFloatingMassMatrix]: unable to retrieve the mass matrix from the reduced model.')
    end
    
    % convert to Matlab format
    M = M_iDyntree.toMatlab;
    
    % debug output
    if KinDynModel.DEBUG
        
        disp('[getFreeFloatingMassMatrix]: debugging outputs...')
                
        % check mass matrix symmetry and positive definiteness
        M_symm_error = norm(M -(M + M')/2);
        M_symm_eig   = eig((M + M')/2);
 
        if M_symm_error > 0.1
            
            error('[getFreeFloatingMassMatrix]: M is not symmetric')
        end
        
        for k = 1:length(M_symm_eig)
            
            if M_symm_eig(k) < 0
                
                error('[getFreeFloatingMassMatrix]: M is not positive definite')
            end
        end       
        disp('[getFreeFloatingMassMatrix]: done.')     
    end 
end
