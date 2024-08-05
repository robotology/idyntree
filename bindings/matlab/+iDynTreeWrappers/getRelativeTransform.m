function frame1_H_frame2 = getRelativeTransform(KinDynModel,frame1Name,frame2Name)

    % GETRELATIVETRANSFORM gets the transformation matrix between two specified
    %                      frames.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT:  frame1_H_frame2 = getRelativeTransform(KinDynModel,frame1Name,frame2Name)
    %
    % INPUTS:  - frame1Name: a string that specifies the frame w.r.t. compute the
    %                        transfomation matrix, or the associated ID;
    %          - frame2Name: a string that specifies the frame w.r.t. compute the
    %                        transfomation matrix, or the associated ID;
    %          - KinDynModel: a structure containing the loaded model and additional info.
    %
    % OUTPUTS: - frame1_H_frame2: [4 x 4] from frame2 to frame1 transformation matrix.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % get the transformation between the frame 1 and 2
    frame1_H_frame2_iDyntree = KinDynModel.kinDynComp.getRelativeTransform(frame1Name,frame2Name);
    frame1_R_frame2_iDyntree = frame1_H_frame2_iDyntree.getRotation;
    framePos_iDyntree        = frame1_H_frame2_iDyntree.getPosition;

    % covert to Matlab format
    frame1_R_frame2          = frame1_R_frame2_iDyntree.toMatlab;
    framePos                 = framePos_iDyntree.toMatlab;
    frame1_H_frame2          = [frame1_R_frame2,framePos;
        0,   0,   0,   1];

    % Debug output
    if KinDynModel.DEBUG

        disp('[getRelativeTransform]: debugging outputs...')

        % frame1_H_frame2 must be a valid transformation matrix
        if size(frame1_H_frame2,1) ~= 4 || size(frame1_H_frame2,2) ~= 4

            error('[getRelativeTransform]: frame1_H_frame2 is not a 4x4 matrix.')
        end

        for ii = 1:4

            if ii < 4

                if abs(frame1_H_frame2(4,ii)) > 0.0001

                    error('[getRelativeTransform]: the last line of frame1_H_frame2 is not [0,0,0,1].')
                end
            else
                if abs(frame1_H_frame2(4,ii)) > 1.0001 || abs(frame1_H_frame2(4,ii)) < 0.9999

                    error('[getRelativeTransform]: the last line of frame1_H_frame2 is not [0,0,0,1].')
                end
            end
        end

        % frame1_R_frame2 = frame1_H_frame2(1:3,1:3) must be a valid rotation matrix
        if det(frame1_H_frame2(1:3,1:3)) < 0.9 || det(frame1_H_frame2(1:3,1:3)) > 1.1

            error('[getRelativeTransform]: frame1_R_frame2 is not a valid rotation matrix.')
        end

        IdentityMatr = frame1_H_frame2(1:3,1:3)*frame1_H_frame2(1:3,1:3)';

        for kk = 1:size(IdentityMatr, 1)

            for jj = 1:size(IdentityMatr, 1)

                if jj == kk

                    if abs(IdentityMatr(kk,jj)) < 0.9 || abs(IdentityMatr(kk,jj)) > 1.1

                        error('[getRelativeTransform]: frame1_R_frame2 is not a valid rotation matrix.')
                    end
                else
                    if abs(IdentityMatr(kk,jj)) > 0.01

                        error('[getRelativeTransform]: frame1_R_frame2 is not a valid rotation matrix.')
                    end
                end
            end
        end
        disp('[getRelativeTransform]: done.')
    end
end
