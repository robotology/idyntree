function [] = visualizerSetup(Visualizer,disableViewInertialFrame,lightDir,cameraPos,cameraTarget)

    % VISUALIZERSETUP modifies the visualization environment according to the 
    %                      user specifications.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % REQUIREMENTS: compile iDyntree with Irrlicht (IDYNTREE_USES_IRRLICHT = ON).
    %
    % FORMAT:  [] = visualizerSetup(Visualizer,disableViewInertialFrame,lightDir,cameraPos,cameraTarget)
    %
    % INPUTS:  - Visualizer: a structure containing the visualizer and its options.
    %          - disableViewInertialFrame: boolean for disabling the view of the 
    %                                      inertial frame;
    %          - lightDir: [3 x 1] vector describing the light direction;
    %          - cameraPos: [3 x 1] vector describing the camera position;
    %          - cameraTarget: [3 x 1] vector describing the camera target;
    % 
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    % disable environmental features  
    if disableViewInertialFrame
        
        enviroment = Visualizer.viz.enviroment();  
        ack        = enviroment.setElementVisibility('root_frame',false);   
        
        % check for errors
        if ~ack
            error('[visualizerSetup]: unable to disable the inertial frame view.')
        end         
    end
    
    % set lights
    sun = Visualizer.viz.enviroment().lightViz('sun');     
    lightDir_idyntree = iDynTree.Direction();     
    lightDir_idyntree.fromMatlab(lightDir); 
    sun.setDirection(lightDir_idyntree);

    % set camera     
    cam = Visualizer.viz.camera();     
    cam.setPosition(iDynTree.Position(cameraPos(1),cameraPos(2),cameraPos(3)));     
    cam.setTarget(iDynTree.Position(cameraTarget(1),cameraTarget(2),cameraTarget(3)));    
    
    % draw the model
    Visualizer.viz.draw();
end
