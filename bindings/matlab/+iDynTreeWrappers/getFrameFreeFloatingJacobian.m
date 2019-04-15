function J_frame = getFrameFreeFloatingJacobian(KinDynModel,frameName)

    % GETFRAMEFREEFLOATINGJACOBIAN gets the free floating jacobian of a 
    %                                   specified frame. 
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  J_frame = getFrameFreeFloatingJacobian(KinDynModel,frameName)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %          - frameName: a string that specifies the frame w.r.t. compute the 
    %                       jacobian matrix, or the associated ID;     
    %
    % OUTPUTS: - J_frame: [6 x ndof+6] frame free floating Jacobian.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

    %% ------------Initialization----------------
    
    % create the matrix that must be populated with the jacobian map
    J_frame_iDyntree = iDynTree.MatrixDynSize(6,KinDynModel.NDOF+6);
    
    % get the free floating jacobian
    ack = KinDynModel.kinDynComp.getFrameFreeFloatingJacobian(frameName,J_frame_iDyntree);
    
    % check for errors
    if ~ack   
        error('[getFrameFreeFloatingJacobian]: unable to get the Jacobian from the reduced model.')
    end
    
    % convert to Matlab format
    J_frame = J_frame_iDyntree.toMatlab;
end
