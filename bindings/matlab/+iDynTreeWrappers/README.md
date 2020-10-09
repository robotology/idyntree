# iDynTree high-level-wrappers

A collection of Matlab/Octave functions that wraps the functionalities of (mainly) the iDyntree class `KinDynComputations`. For further information on the single wrapper, refer to the description included in each function.

## Motivations

For a Matlab/Octave user, it may be sometimes counterintuitive to use C++ based formalism inside Matlab/Octave. Furthermore, there are common iDyntree functions that require several lines of code in order to be used correctly. E.g. see the `getRobotState` function:

```
    basePose_iDyntree   = iDynTree.Transform();
    jointPos_iDyntree   = iDynTree.VectorDynSize(KinDynModel.NDOF);
    baseVel_iDyntree    = iDynTree.Twist();
    jointVel_iDyntree   = iDynTree.VectorDynSize(KinDynModel.NDOF);
    gravityVec_iDyntree = iDynTree.Vector3();

    KinDynModel.kinDynComp.getRobotState(basePose_iDyntree,jointPos_iDyntree,baseVel_iDyntree,jointVel_iDyntree,gravityVec_iDyntree);

    baseRotation_iDyntree = basePose_iDyntree.getRotation;
    baseOrigin_iDyntree   = basePose_iDyntree.getPosition;

    baseRotation = baseRotation_iDyntree.toMatlab;
    baseOrigin   = baseOrigin_iDyntree.toMatlab;
    jointPos     = jointPos_iDyntree.toMatlab;
    baseVel      = baseVel_iDyntree.toMatlab;
    jointVel     = jointVel_iDyntree.toMatlab;
    basePose     = [baseRotation, baseOrigin;
                       0,  0,  0,  1];
```

The purpose of the wrapper is therefore to provide a simpler and easy-to-use interface for Matlab/Octave users who wants to use iDyntree inside Matlab/Octave, also helping in designing code which is less error-prone and easier to debug (e.g. in case the interface of an iDyntree function will change in the future).

## KinDynComputations class

### Load the reduced model

- [loadReducedModel](loadReducedModel.m)

### Retrieve data from the model

- [getJointPos](getJointPos.m)
- [getJointVel](getJointVel.m)
- [getCentroidalTotalMomentum](getCentroidalTotalMomentum.m)
- [getRobotState](getRobotState.m)
- [getWorldBaseTransform](getWorldBaseTransform.m)
- [getBaseTwist](getBaseTwist.m)
- [getModelVel](getModelVel.m)
- [getFrameVelocityRepresentation](getFrameVelocityRepresentation.m)
- [getNrOfDegreesOfFreedom](getNrOfDegreesOfFreedom.m)
- [getFloatingBase](getFloatingBase.m)
- [getFrameIndex](getFrameIndex.m)
- [getFrameName](getFrameName.m)
- [getWorldTransform](getWorldTransform.m)
- [getWorldTransformsAsHomogeneous](getWorldTransformsAsHomogeneous.m)
- [getRelativeTransform](getRelativeTransform.m)
- [getRelativeJacobian](getRelativeJacobian.m)
- [getFreeFloatingMassMatrix](getFreeFloatingMassMatrix.m)
- [getFrameBiasAcc](getFrameBiasAcc.m)
- [getFrameFreeFloatingJacobian](getFrameFreeFloatingJacobian.m)
- [getCenterOfMassPosition](getCenterOfMassPosition.m)
- [generalizedBiasForces](generalizedBiasForces.m)
- [generalizedGravityForces](generalizedGravityForces.m)
- [getCenterOfMassJacobian](getCenterOfMassJacobian.m)
- [getCenterOfMassVelocity](getCenterOfMassVelocity.m)

### Set the model-related quantities

- [setJointPos](setJointPos.m)
- [setWorldBaseTransform](setWorldBaseTransform.m)
- [setFrameVelocityRepresentation](setFrameVelocityRepresentation.m)
- [setFloatingBase](setFloatingBase.m)
- [setRobotState](setRobotState.m)

## Visualizer class

Not proper wrappers, they wrap more than one method of the class each. **Requirements:** compile iDyntree with Irrlicht `(IDYNTREE_USES_IRRLICHT = ON)`.

**Warning**: the visualizer class may be deprecated in a future release.

- [initializeVisualizer](initializeVisualizer.m)
- [visualizerSetup](visualizerSetup.m)
- [updateVisualizer](updateVisualizer.m)

## Matlab Native visualization

Not actual wrappers, they use the iDynTreeWrappers in combination with the MATLAB patch plotting functions to visualize the robot.

**Disclaimers**:

1) This visualization **has not been tested** with Octave.
2) At the moment, there is **no support** for `.dae` mesh file format.

- [prepareVisualization](prepareVisualization.m)
- [updateVisualization](updateVisualization.m)
- [getMeshes](getMeshes.m)
- [plotMeshInWorld](plotMeshInWorld.m)
- [modifyLinkVisual](modifyLinkVisual.m)
- [modifyLinksVisualization](modifyLinksVisualization.m)
