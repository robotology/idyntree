function [] = setWorldBaseTransform(KinDynModel,basePose)

    % SETWORLDBASETRANSFORM sets the joints configuration for kino-dynamic
    %                       computations.
    %
    % This matlab function wraps a functionality of the iDyntree library.
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT: [] = setWorldBaseTransform(KinDynModel,basePose)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %          - basePose: [4 x 4] from base frame to world frame transform;
    %
    % Author : Lorenzo Moretti (lorenzo.moretti@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause

    %% ------------Initialization----------------

    % Debug input
    if KinDynModel.DEBUG

        disp('[setWorldBaseTransform]: debugging inputs...')

        % basePose must be a valid transformation matrix
        if size(basePose,1) ~= 4 || size(basePose,2) ~= 4

            error('[setWorldBaseTransform]: basePose is not a 4x4 matrix.')
        end

        for ii = 1:4

            if ii < 4

                if abs(basePose(4,ii)) > 0.0001

                    error('[setWorldBaseTransform]: the last line of basePose is not [0,0,0,1].')
                end
            else
                if abs(basePose(4,ii)) > 1.0001 || abs(basePose(4,ii)) < 0.9999

                    error('[setWorldBaseTransform]: the last line of basePose is not [0,0,0,1].')
                end
            end
        end

        % baseRotation = basePose(1:3,1:3) must be a valid rotation matrix
        if det(basePose(1:3,1:3)) < 0.9 || det(basePose(1:3,1:3)) > 1.1

            error('[setWorldBaseTransform]: baseRotation is not a valid rotation matrix.')
        end

        IdentityMatr = basePose(1:3,1:3)*basePose(1:3,1:3)';

        for kk = 1:size(IdentityMatr, 1)

            for jj = 1:size(IdentityMatr, 1)

                if jj == kk

                    if abs(IdentityMatr(kk,jj)) < 0.9 || abs(IdentityMatr(kk,jj)) > 1.1

                        error('[setWorldBaseTransform]: baseRotation is not a valid rotation matrix.')
                    end
                else
                    if abs(IdentityMatr(kk,jj)) > 0.01

                        error('[setWorldBaseTransform]: baseRotation is not a valid rotation matrix.')
                    end
                end
            end
        end

        disp('[setWorldBaseTransform]: done.')
    end

    % define the quantities required to set the floating base
    baseRotation_iDyntree = iDynTree.Rotation();
    baseOrigin_iDyntree   = iDynTree.Position();
    basePose_iDyntree     = iDynTree.Transform();

    % set the element of the rotation matrix and of the base
    % position vector
    for k = 0:2

        baseOrigin_iDyntree.setVal(k,basePose(k+1,4));

        for j = 0:2

            baseRotation_iDyntree.setVal(k,j,basePose(k+1,j+1));
        end
    end

    % add the rotation matrix and the position to basePose_iDyntree
    basePose_iDyntree.setRotation(baseRotation_iDyntree);
    basePose_iDyntree.setPosition(baseOrigin_iDyntree);

    % set the world base transform
    ack = KinDynModel.kinDynComp.setWorldBaseTransform(basePose_iDyntree);

    % check for errors
    if ~ack
        error('[setWorldBaseTransform]: unable to set the world base transform.')
    end
end
