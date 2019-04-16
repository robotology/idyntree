function J_frameVel = getRelativeJacobian(KinDynModel,frameVelID,frameRefID)

    % GETRELATIVEJACOBIAN gets the relative jacobian, i.e. the matrix that
    %                          maps the velocity of frameVel expressed w.r.t.
    %                          frameRef, to the joint velocity. 
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  J_frameVel = getRelativeJacobian(KinDynModel,frameVelID,frameRefID)
    %
    % INPUTS:  - frameRefID: a number that specifies the frame w.r.t. the velocity
    %                        of frameVel is expressed;
    %          - frameVelID: a number that specifies the frame whose velocity
    %                        is the one mapped by the jacobian;
    %          - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - J_frameVel: [6 x ndof] frameVel Jacobian.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------

    % create the matrix that must be populated with the jacobian map
    J_frameVel_iDyntree = iDynTree.MatrixDynSize(6,KinDynModel.NDOF);
    
    % get the relative jacobian
    ack = KinDynModel.kinDynComp.getRelativeJacobian(frameVelID,frameRefID,J_frameVel_iDyntree);  
    
    % check for errors
    if ~ack  
        error('[getRelativeJacobian]: unable to get the relative jacobian from the reduced model.')
    end
    
    % covert to Matlab format
    J_frameVel = J_frameVel_iDyntree.toMatlab; 
end
