function []=modifyLinksVisualization(Visualizer,varargin)
    % Allows the modifications of the meshes of the selected links. By default it will modify all objects in the Visualizer variable
    %   - `modifyLinksVisualization` : Updates the figure image with the new robot state. To be launched after `setRobotState` has been used.
    %       - Inputs:
    %             - `Visualizer` : variable output from the `prepareVisualization` function. It contains the relevant variables `linkNames`,`transforms` and `NOBJ`.
    %                 - `linkNames`  : variable that contains the link names.
    %                 - `meshHandles` : variable that has the handles of the mesh objects.
    %                 - `NOBJ` : variable that contains the number of visual objects to update.
    %       - Optional Inputs:
    %             - `linksToModify`  : Cell array containing the names of the links to modify
    %             - `linksIndices`   : array containing the indices of the links to modify
    % :exclamation:   Note: all extra variables are sent to `modifyLinkVisual`
    %
    % Author : Francisco Andrade (franciscojavier.andradechavez@iit.it)
    %
    % SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
    % SPDX-License-Identifier: BSD-3-Clause
    %% input parser section
    p = inputParser;
    p.StructExpand = false;
    p.KeepUnmatched= true;
    % Default values
    default_linksToModify={'all'};
    default_linksIndices=-99;
    % add parameters and validity funcitons
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
                notInModel = length(indices_temp)-length(indices);
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
