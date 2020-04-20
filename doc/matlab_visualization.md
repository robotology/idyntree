# How to use Matlab Visualization
The MATLAB Visualization uses the MATLAB capabilities for displaying meshes and figures and the floating-base kinematics of iDynTree to get the new poses of the links.
It uses the [iDynTreeWrappers](../bindings/matlab/%2BiDynTreeWrappers) to handle the model and obtaining the required transforms.

**Disclaimers**:
- This visualization has not been tested with Octave.
- At the moment there is no support for .dae mesh files.

## functions
### Main functions
- `prepareVisualization` : Creates the figures, loads the meshes, handles some visual effects, and creates the transform objects
    - Iputs:
        - `KinDynModel` : iDynTreeWrappers main variable. Contains the model.
        - `meshFilePrefix` : Path in which we can find the meshes. As an example the path to the mesh in a iCub urdf is `'package://iCub/meshes/simmechanics/sim_sea_2-5_root_link_prt-binary.stl'
`. `meshFilePrefix` should replace package to allow to find the rest of the path.
    - Outputs:
        - `transforms` : Is the transform objects array. There is one transform for each link
        - `linkNames`  : The name of the links in the same order of the transforms
        - `map`       : Cell array having both the names of the meshes and the associated link
        - `linkMeshInfo` : Contains the link name and a struct (meshInfo) that contains the name of file or if is a simple geometry, the triangulation ( edges and vertices of the mesh ) and the link to geometry transform.
        - `meshHandles` : Has the handles of the mesh objects.
        - `parent`     : Contains the axes object of the parent figure.
        - `mainHandler`: Is the handle of the overall figure.

- `updateVisualization` : Updates the figure image with the new robot state. To be launched after `setRobotState` has been used.
    - Inputs:
          - `KinDynModel`: iDynTreeWrappers main variable. Contains the model.
          - `linkNames`  : The `linkNames` variable output from the `prepareVisualization` function.
          - `transforms` : The `transforms` variable output from the `prepareVisualization` function.

### Other functions
- `getMeshes` : Gets the mesh information for each link in the model.
    - Iputs:
        - `model` : iDynTree model loaded form a URDF.
        - `meshFilePrefix` : Path in which we can find the meshes. As an example the path to the mesh in a iCub urdf is `'package://iCub/meshes/simmechanics/sim_sea_2-5_root_link_prt-binary.stl'
`. `meshFilePrefix` should replace package to allow to find the rest of the path.
    - Outputs:
        - `map`        : Cell array having both the names of the meshes and the associated link
        - `linkMeshInfo` : Struct array that contain the link name and a struct (`meshInfo`) that contains the name of file or if is a simple geometry, the triangulation ( edges and vertices of the mesh ) and the link to geometry transform.

- `plotMeshInWorld` : Gets the mesh information for each link in the model.
            - Iputs:
                - `linkMeshInfo` : An individual `linkMeshInfo` from the array output variable from `getMeshes` function.
                - `w_H_link` : Homogeneous transform from the world to the link
            - Outputs:
                - `meshHandles`  : Struct that contains all the created handles for the meshes. There can be more than one mesh for each link.
                - `transform`            : The transform object for the link
