# How to use Matlab Visualization
The MATLAB Visualization uses the MATLAB capabilities for displaying meshes and figures and the floating-base kinematics of iDynTree to get the new poses of the links.
It uses the [iDynTreeWrappers](../bindings/matlab/%2BiDynTreeWrappers) to handle the model and obtaining the required transforms.

**Disclaimers**:
- This visualization has not been tested with Octave.
- At the moment there is no support for .dae mesh files.

## functions
### Main functions
- `prepareVisualization` : Creates the figures, loads the meshes, handles some visual effects, and creates the transform objects
    - Inputs:
        - `KinDynModel` : iDynTreeWrappers main variable. Contains the model.
        - `meshFilePrefix` : Path in which we can find the meshes. As an example the path to the mesh in a iCub urdf is `'package://iCub/meshes/simmechanics/sim_sea_2-5_root_link_prt-binary.stl'
`. `meshFilePrefix` should replace package to allow to find the rest of the path. If the value is "", the standard iDynTree workflow of locating the mesh via the ExternalMesh.getFileLocationOnLocalFileSystem method is used.
        - `varargin`  : Variable that allows to add option configuration parameters. Admitted options are:
            - `view` : Selects the angle in which the figure is seen.
            - `material` : Selects effect with which the patch is rendered. Options are : 'dull','metal','shiny';            
            - `debug` : Enables having extra information for debugging purposes.
            - `groundOn` : Enables showing a plane as the ground.
            - `groundColor` : Selects the color of the ground.
            - `groundTransparency` : Selects the transparency of the ground.
            - `groundFrame` : Selects the frame in which the ground is attached.
:exclamation:   Note: all extra variables are sent to `plotMeshInWorld`
    - Outputs:
        - `Visualizer` : Struct containing the following fields
            - `transforms` : Is the transform objects array. There is one transform for each link
            - `linkNames`  : The name of the links in the same order of the transforms
            - `linkNames_idyn`  : The idyntree string vector equivalent to `linkNames`
            - `NOBJ` : Contains the number of visual objects in the visualizer            
            - `meshHandles` : Has the handles of the mesh objects.
            - `parent`     : Contains the axes object of the parent figure.
            - `mainHandler`: Is the handle of the overall figure.
            - `DEBUG` : Flag stating if the debug mode was set or not.
            Fields if debug mode is on:
            - `map`       : Cell array having both the names of the meshes and the associated link
            - `linkMeshInfo` : Contains the link name and a struct (meshInfo) that contains the name of file or if is a simple geometry, the triangulation ( edges and vertices of the mesh ) and the link to geometry transform.

- `updateVisualization` : Updates the figure image with the new robot state. To be launched after `setRobotState` has been used.
    - Inputs:
          - `KinDynModel`: iDyntreewrappers main variable. Contains the model.
          - `Visualizer` : variable output from the `prepareVisualization` function. It contains the relevant variables `linkNames`,`meshHandles` and `NOBJ`.
              - `linkNames`  : variable that contains the link names.
              - `transforms` : variable that contains transform objects that are parent of the meshes.
              - `NOBJ` : variable that contains the number of visual objects to update.

  - `modifyLinksVisualization` : Allows the modifications of the meshes of the selected links. By default it will modify all objects in the `Visualizer` variable
      - Inputs:
            - `Visualizer` : variable output from the `prepareVisualization` function. It contains the relevant variables `linkNames`,`transforms` and `NOBJ`.
                - `linkNames`  : variable that contains the link names.
                - `meshHandles` : variable that has the handles of the mesh objects.
                - `NOBJ` : variable that contains the number of visual objects to update.
      - Optional Inputs:
            - `linksToModify`  : Cell array containing the names of the links to modify
            - `linksIndices`   : array containing the indices of the links to modify
:exclamation:   Note: all extra variables are sent to `modifyLinkVisual`

### Other functions
- `getMeshes` : Gets the mesh information for each link in the model.
    - Inputs:
        - `model` : iDynTree model loaded form a URDF.
        - `meshFilePrefix` : Path in which we can find the meshes. As an example the path to the mesh in a iCub urdf is `'package://iCub/meshes/simmechanics/sim_sea_2-5_root_link_prt-binary.stl' 
`. `meshFilePrefix` should replace package to allow to find the rest of the path. If the value is "", the standard iDynTree workflow of locating the mesh via the ExternalMesh.getFileLocationOnLocalFileSystem method is used.
    - Outputs:
        - `map`        : Cell array having both the names of the meshes and the associated link
        - `linkMeshInfo` : Struct array that contain the link name and a struct (`meshInfo`) that contains the name of file or if is a simple geometry, the triangulation ( edges and vertices of the mesh ) and the link to geometry transform.

- `plotMeshInWorld` : Gets the mesh information for each link in the model.
    - Inputs:
        - `linkMeshInfo` : An individual `linkMeshInfo` from the array output variable from `getMeshes` function.
        - `w_H_link` : Homogeneous transform from the world to the link
    - Optional Inputs:
        - `color` : Selects the color of the meshes.
        - `transparency` : Sets the alpha value of the patch. Is the
        level of transparency between 0 fully transparent and 1 solid.
        - `wireframe_rendering` : the reduction ratio of faces to
        wireframe. Follows convention of reducepatch.
        - `style` : Selects the style of display of meshes, either fullMesh or wireframe.
    - Outputs:
        - `meshHandles`  : Struct that contains all the created handles for the meshes. There can be more than one mesh for each link.
        - `transform`    : The transform object for the link

- `modifyLinkVisual` : Modifies a single `meshHandle` with the specified changes.
    - Inputs:
        - `meshHandle` : Struct containing the patch type of variable ( `modelMesh` field ) and the original information of the mesh in `fullMesh_bckup` field.
    - Optional Inputs:
        - `color` : Selects the color of the meshes.
        - `transparency` : Sets the alpha value of the patch. Is the
        level of transparency between 0 fully transparent and 1 solid.
        - `wireframe_rendering` : the reduction ratio of faces to
        wireframe. Follows convention of reducepatch.
        - `style` : Selects the style of display of meshes, either fullMesh or wireframe.
