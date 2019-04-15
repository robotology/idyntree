function JDot_nu_frame = getFrameBiasAcc(KinDynModel,frameName)

    % GETFRAMEBIASACC gets the bias accelerations of a specified frame. 
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
    %
    % FORMAT:  JDot_nu_frame = getFrameBiasAcc(KinDynModel,frameName)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %          - frameName: a string that specifies the frame w.r.t. compute the 
    %                       bias accelerations, or the associated ID;     
    %
    % OUTPUTS: - JDot_nu_frame: [6 x ndof+6] frame bias accelerations.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Nov 2018

    %% ------------Initialization----------------
    
    % get the bias acc
    JDot_nu_frame_iDyntree = KinDynModel.kinDynComp.getFrameBiasAcc(frameName);
    
    % convert to Matlab format
    JDot_nu_frame = JDot_nu_frame_iDyntree.toMatlab;
end
