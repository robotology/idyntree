function [] = modifyLinksVisualization(Visualizer, varargin)

    % MODIFYLINKSVISUALIZATION Allows the modifications of the meshes of the 
    %                          selected links. By default it will modify all 
    %                          objects in the Visualizer variable.
    %
    % NOTE: to be launched after setRobotState has been used.
    %
    % FORMAT: [] = modifyLinksVisualization(Visualizer, varargin)
    %
    % INPUTS: - Visualizer: variable output from the prepareVisualization
    %                       function. It contains the relevant variables 
    %                       linkNames, transforms and NOBJ.
    %
    %            * linkNames: variable that contains the link names.
    %            * meshHandles: variable that has the handles of the mesh objects.
    %            * NOBJ: variable that contains the number of visual objects to update.
    %
    % OPTIONAL INPUTS: - linksToModify: Cell array containing the names
    %                                   of the links to modify.
    %                  - linksIndices: Array containing the indices of
    %                                  the links to modify.
    %
    % :exclamation: Note: all extra variables are sent to modifyLinkVisual.
    %
    % Author : Francisco Andrade (franciscojavier.andradechavez@iit.it)
    %          Modified by: Gabriele Nava (gabriele.nava@iit.it)
    %    
    % Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
    % This software may be modified and distributed under the terms of the
    % GNU Lesser General Public License v2.1 or any later version.

    %% ------------Initialization----------------

    % Input parser section
    p               = inputParser;
    p.StructExpand  = false;
    p.KeepUnmatched = true;
    
    % Default values
    default_linksToModify = {'all'};
    default_linksIndices  = -99;
    
    % Add parameters and validity funcitons
    addRequired(p,'structInput');
    addParameter(p,'linksToModify',default_linksToModify,@(x) iscell(x));
    addParameter(p,'linksIndices',default_linksIndices,@(x)isnumeric(x) && all(x>0) && ~any(x>Visualizer.NOBJ));

    % Parse inputs
    parse(p,Visualizer,varargin{:});
    options = p.Results;

    if ~any(ismember(p.UsingDefaults,'linksToModify')) || ~any(ismember(p.UsingDefaults,'linksIndices'))
    
        % Select the indices of the links to modify
        if ~any(ismember(p.UsingDefaults,'linksIndices'))
        
            indices = options.linksIndices;
        else
            [~,indices_temp] = ismember(options.linksToModify,Visualizer.linkNames);
            indices = nonzeros(indices_temp);
            
            if length(indices_temp) ~= length(indices)
                
                notInModel         = length(indices_temp)-length(indices);
                indices_notInModel = find(indices_temp==0);
                warndlg(sprintf('The list of links to modify contains %d names (positions [ %s ] ) that are not in the list of links of the model',notInModel,num2str(indices_notInModel')));
            end
        end
    else
        % By default modify all links
        indices = 1:Visualizer.NOBJ;
    end

    for it = 1:length(indices)
    
        iDynTreeWrappers.modifyLinkVisual(Visualizer.meshHandles(indices(it)),varargin{:});
    end
end
