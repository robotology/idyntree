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
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % get the bias acc
    JDot_nu_frame_iDyntree = KinDynModel.kinDynComp.getFrameBiasAcc(frameName);

    % convert to Matlab format
    JDot_nu_frame = JDot_nu_frame_iDyntree.toMatlab;
end
