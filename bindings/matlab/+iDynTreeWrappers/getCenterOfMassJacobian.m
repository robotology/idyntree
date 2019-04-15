function J_CoM = getCenterOfMassJacobian(KinDynModel)

    % GETCENTEROFMASSJACOBIAN retrieves the CoM jacobian.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  J_CoM = getCenterOfMassJacobian(KinDynModel)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - J_CoM: [3 x ndof+6] CoM free floating Jacobian.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

    %% ------------Initialization----------------
    
    % create the matrix that must be populated with the jacobian map
    J_CoM_iDyntree = iDynTree.MatrixDynSize(3,KinDynModel.NDOF+6);
    
    % get the free floating jacobian
    ack = KinDynModel.kinDynComp.getCenterOfMassJacobian(J_CoM_iDyntree);
    
    % check for errors
    if ~ack    
        error('[getCenterOfMassJacobian]: unable to get the CoM Jacobian from the reduced model.')
    end
    
    % convert to Matlab format
    J_CoM = J_CoM_iDyntree.toMatlab;
end
