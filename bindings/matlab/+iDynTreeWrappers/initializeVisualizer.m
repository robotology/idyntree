function Visualizer = initializeVisualizer(KinDynModel,debugMode)

    % INITIALIZEVISUALIZER opens the iDyntree visualizer and loads the reduced
    %                           model into the visualizer.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: https://github.com/robotology/idyntree
    %
    % REQUIREMENTS: compile iDyntree with Irrlicht (IDYNTREE_USES_IRRLICHT = ON).
    %
    % FORMAT:  Visualizer = initializeVisualizer(KinDynModel,debugMode)
    %
    % INPUTS:  - KinDynModel: a structure containing the loaded model and additional info.
    %          - debugMode: if TRUE, the visualizer is used in "debug" mode;
    %
    % OUTPUTS: - Visualizer: a structure containing the visualizer and its options.
    % 
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    %
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------
    
    Visualizer.viz   = iDynTree.Visualizer();
    
    % load the model in the visualizer
    ack = Visualizer.viz.addModel(KinDynModel.kinDynComp.model(),'viz1');
    
    % check for errors
    if ~ack   
        error('[initializeVisualizer]: unable to load the model in the visualizer.')
    end  

    % draw the model
    Visualizer.viz.draw();
    
    % if DEBUG option is set to TRUE, all the wrappers related to the
    % iDyntree visualizer will be run in DEBUG mode.
    Visualizer.DEBUG = debugMode;    
end
