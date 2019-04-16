function [] = setRobotState(varargin)

    % SETROBOTSTATE sets the system state. The system state is composed of:
    %                   
    %                    - joints configuration and velocity plus gravity vector 
    %                      for fixed-base systems; 
    %                    - base pose and velocity, joints configuration and 
    %                      velocity plus gravity vector for floating-base systems. 
    %
    %                    The gravity vector expresses the gravity acceleration 
    %                    in the inertial frame. 
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % FORMAT: Floating base system:
    %
    %         [] = setRobotState(KinDynModel,basePose,jointPos,baseVel,jointVel,gravAcc)
    %
    %         Fixed base system:
    %
    %         [] = setRobotState(KinDynModel,jointPos,jointVel,gravAcc)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %          - basePose: [4 x 4] from base frame to world frame transform;
    %          - jointPos: [nDof x 1] vector representing the joints configuration 
    %                      in radians;
    %          - baseVel: [6 x 1] vector of base velocities [lin, ang];
    %          - jointVel: [ndof x 1] vector of joints velocities;
    %          - gravAcc: [3 x 1] vector of the gravity acceleration in the
    %                             inertial frame.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------

    KinDynModel = varargin{1};
    
    % check the number of inputs
    switch nargin
       
        case 6
            
            basePose = varargin{2};
            jointPos = varargin{3};
            baseVel  = varargin{4};
            jointVel = varargin{5};
            gravAcc  = varargin{6};
            
            % Debug inputs
            if KinDynModel.DEBUG
        
                disp('[setRobotState]: debugging inputs...')
        
                % basePose must be a valid transformation matrix
                if size(basePose,1) ~= 4 || size(basePose,2) ~= 4
            
                    error('[setRobotState]: basePose is not a 4x4 matrix.')
                end
        
                for ii = 1:4
                    
                    if ii < 4
                        
                        if abs(basePose(4,ii)) > 0.0001 
            
                            error('[setRobotState]: the last line of basePose is not [0,0,0,1].')
                        end
                    else
                        if abs(basePose(4,ii)) > 1.0001 || abs(basePose(4,ii)) < 0.9999
            
                            error('[setRobotState]: the last line of basePose is not [0,0,0,1].')
                        end
                    end
                end
        
                % baseRotation = basePose(1:3,1:3) must be a valid rotation matrix
                if det(basePose(1:3,1:3)) < 0.9 || det(basePose(1:3,1:3)) > 1.1
            
                    error('[setRobotState]: baseRotation is not a valid rotation matrix.')
                end
        
                IdentityMatr = basePose(1:3,1:3)*basePose(1:3,1:3)';
        
                for kk = 1:size(IdentityMatr, 1)
            
                    for jj = 1:size(IdentityMatr, 1)
                
                        if jj == kk
                    
                            if abs(IdentityMatr(kk,jj)) < 0.9 || abs(IdentityMatr(kk,jj)) > 1.1
                        
                                error('[setRobotState]: baseRotation is not a valid rotation matrix.')
                            end
                        else
                            if abs(IdentityMatr(kk,jj)) > 0.01
                        
                                error('[setRobotState]: baseRotation is not a valid rotation matrix.')
                            end
                        end
                    end   
                end   
            
                % debug gravity vector size
                if length(gravAcc) ~= 3
            
                     error('[setRobotState]: the length of gravAcc vector is not 3.')
                end 
                
                % check base velocity vector size
                if length(baseVel) ~= 6
            
                     error('[setRobotState]: the length of baseVel is not 6.')
                end
                
                % check joints position vector size
                if length(jointPos) ~= KinDynModel.NDOF
            
                    error('[setRobotState]: the length of jointPos is not KinDynModel.NDOF')
                end
                
                % check joints velocity vector size
                if length(jointVel) ~= KinDynModel.NDOF
            
                    error('[setRobotState]: the length of jointVel is not KinDynModel.NDOF')
                end
                
                disp('[setRobotState]: done.')     
            end
            
            % define the quantities required to set the floating base
            baseRotation_iDyntree = iDynTree.Rotation();
            baseOrigin_iDyntree   = iDynTree.Position();
            basePose_iDyntree     = iDynTree.Transform();
            baseVel_iDyntree      = iDynTree.Twist();
            
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
            
            % set the base velocities
            for k = 0:5
                
                baseVel_iDyntree.setVal(k,baseVel(k+1));
            end
            
        case 4
            
            jointPos = varargin{2}; 
            jointVel = varargin{3};
            gravAcc  = varargin{4};
            
            % Debug inputs
            if KinDynModel.DEBUG
        
                disp('[setRobotState]: debugging inputs...')
                    
                % debug gravity vector
                if length(gravAcc) ~= 3
            
                     error('[setRobotState]: the length of gravAcc vector is not 3.')
                end
                
                % check joints position vector size
                if length(jointPos) ~= KinDynModel.NDOF
            
                    error('[setRobotState]: the length of jointPos is not KinDynModel.NDOF')
                end
                
                % check joints velocity vector size
                if length(jointVel) ~= KinDynModel.NDOF
            
                    error('[setRobotState]: the length of jointVel is not KinDynModel.NDOF')
                end
                
                disp('[setRobotState]: done.')     
            end
            
        otherwise        
            error('[setRobotState]: wrong number of inputs.')
    end
    
    % define all the remaining quantities required for setting the system state
    jointPos_iDyntree   = iDynTree.VectorDynSize(KinDynModel.NDOF);
    jointVel_iDyntree   = iDynTree.VectorDynSize(KinDynModel.NDOF);
    gravityVec_iDyntree = iDynTree.Vector3();
         
    % set joints position and velocity
    for k = 0:length(jointPos)-1
        
        jointVel_iDyntree.setVal(k,jointVel(k+1));
        jointPos_iDyntree.setVal(k,jointPos(k+1));
    end
    
    % set the gravity vector
    for k = 0:2
        
       gravityVec_iDyntree.setVal(k,gravAcc(k+1));       
    end
    
    % set the current robot state
    switch nargin
        
        case 4
            
            ack = KinDynModel.kinDynComp.setRobotState(jointPos_iDyntree,jointVel_iDyntree,gravityVec_iDyntree);
            
        case 6
            
            ack = KinDynModel.kinDynComp.setRobotState(basePose_iDyntree,jointPos_iDyntree,baseVel_iDyntree,jointVel_iDyntree,gravityVec_iDyntree);
    end
    
    % check for errors
    if ~ack   
        error('[setRobotState]: unable to set the robot state.')
    end  
end
