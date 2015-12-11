import iDynTree
import numpy as np

# First we create the regressor generator,
# the class to compute the dynamics regressors
regrGen = iDynTree.DynamicsRegressorGenerator();

# Secondly we load the structure of the robot
# (kinematic and inertial parameters) and the
# structure of the sensors from an URDF file
regrGen.loadRobotAndSensorsModelFromFile('icub.urdf');

# Then we load the structure of the regressor
regrXml =  '<regressor>' \
           '  <subtreeBaseDynamics>' \
           '   <FTSensorLink>r_upper_arm</FTSensorLink>' \
           '   </subtreeBaseDynamics>' \
           '</regressor>'
regrGen.loadRegressorStructureFromString(regrXml);

# once we loaded the robot parameters and the regressor structure,
# we can use the regressor generator to get information about the
# generated regressor

# Get the number of internal degrees of freedom of the considered robot
dof = regrGen.getNrOfDegreesOfFreedom()

# Get the number of parameters of the regressor (most of them will be zero)
# TODO : put in the regressor only the relevant parameters
params = regrGen.getNrOfParameters()

# Get the number of outputs of the regressor
# Given that we are considering only the base dynamics
# of a subtree, we will have just 6 outputs (3 force, 3 torques)
outs = regrGen.getNrOfOutputs()

# We can now create the input for the regressor:
# joint position, velocities and acceleration
# and gravity acceleration at the base frame
# (you can get the base link with the regrGen.getBaseLink() method)

# If you are unsure of the assumed serialization of the DOFs, you can use
# the regrGen.getDescriptionOfDegreesOfFreedom() method
qj = iDynTree.VectorDynSize(dof);
dqj = iDynTree.VectorDynSize(dof);
ddqj = iDynTree.VectorDynSize(dof);
gravity = iDynTree.Twist();

# Currently the smooth conversion between Matlab and iDynTree vector and
# matrices is still a TODO, so for now we have to rely on the setVal/getVal
# methods
gravity.setVal(0,0.0);
gravity.setVal(1,0.0);
gravity.setVal(2,-9.81);

# Here we should fill the q/dqj/ddqj with measured values

# Note that we are using the "fixed base" version of setRobotState
regrGen.setRobotState(qj,dqj,ddqj,gravity);

# TODO: set sensor values

# Now we can compute the regressor
regressor  = iDynTree.MatrixDynSize(outs,params);
knownTerms = iDynTree.VectorDynSize(outs);
cadParams  = iDynTree.VectorDynSize(params);

# We can get the current model parameters (probably extracted from CAD)
regrGen.getModelParameters(cadParams)

# We want to set the measure for the `l_arm_ft_sensor`
sensorMeasure = iDynTree.Wrench();
sensorMeasure.setVal(0,0.0);
sensorMeasure.setVal(1,0.0);
sensorMeasure.setVal(2,10.0);
sensorMeasure.setVal(3,0.4);
sensorMeasure.setVal(4,0.6);
sensorMeasure.setVal(5,0.5);

sensorIndex = \
    regrGen.getSensorsModel().getSensorIndex(iDynTree.SIX_AXIS_FORCE_TORQUE,'r_arm_ft_sensor');
regrGen.getSensorsMeasurements().setMeasurement(iDynTree.SIX_AXIS_FORCE_TORQUE,sensorIndex,sensorMeasure);

regrGen.computeRegressor(regressor,knownTerms);

# Convert data to numpy
A = np.fromstring(regressor.toString(), sep=' ').reshape(outs, params);
b = np.fromstring(knownTerms.toString(), sep=' ');
phi_cad = np.fromstring(cadParams.toString(), sep=' ');

# predict sensor measurement using CAD inertial parameters
sensor_prediction_cad = np.dot(A, phi_cad);

# We can then print the computed regressor
print "Regressor:"
print regressor.toString()
print "Known terms:"
print knownTerms.toString()
print "\nKnown terms predicted with cad parameters:"
print sensor_prediction_cad
print "\nCAD parameters:"
print cadParams.toString()
