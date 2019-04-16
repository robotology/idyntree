function JDot_nu_frame = getFrameBiasAcc(KinDynModel,frameName)

    % GETFRAMEBIASACC gets the bias accelerations of a specified frame. 
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  JDot_nu_frame = getFrameBiasAcc(KinDynModel,frameName)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %          - frameName: a string that specifies the frame w.r.t. compute the 
    %                       bias accelerations, or the associated ID;     
    %
    % OUTPUTS: - JDot_nu_frame: [6 x 6+ndof] frame bias accelerations.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    % get the bias acc
    JDot_nu_frame_iDyntree = KinDynModel.kinDynComp.getFrameBiasAcc(frameName);
    
    % convert to Matlab format
    JDot_nu_frame = JDot_nu_frame_iDyntree.toMatlab;
end
