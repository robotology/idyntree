function Visualizer = initializeVisualizer(KinDynModel,debugMode)

    % INITIALIZEVISUALIZER opens the iDyntree visualizer and loads the reduced
    %                           model into the visualizer.
    %
    % This matlab function wraps a functionality of the iDyntree library.                     
    % For further info see also: http://wiki.icub.org/codyco/dox/html/idyntree/html/
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
    % Genova, Nov 2018

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
