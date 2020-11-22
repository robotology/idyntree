
% First we create the SensorsList object,
% that will be parsed from URDF
sensList = iDynTree.SensorsList();

% Secondly we load the structure of the sensors list
% from URDF
mdlLoader = iDynTree.ModelLoader();
mdlLoader.loadModelFromFile('./icub.urdf');

sensList = mdlLoader.sensors();

% We can check the number of FT sensors loaded
nrOfFTSensors = sensList.getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);
fprintf('The loaded model has %d FT sensors\n',nrOfFTSensors)

% We can print the information relative to the sensors
for sensIndex = 0:(nrOfFTSensors-1)

    % Getting the sensor
    FTSens = sensList.getSixAxisForceTorqueSensor(sensIndex);

    % Print info
    fprintf('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n');
    fprintf('Sensor %s has index %d\n',FTSens.getName(),sensIndex);
    fprintf(' is attached to joint %s\n',FTSens.getParent());

    % Get the direction of the measure
    if( FTSens.getAppliedWrenchLink() == FTSens.getFirstLinkIndex() )
        fprintf(' it measures the wrench that link %s is applying on link %s\n',FTSens.getSecondLinkName(),FTSens.getFirstLinkName());
    else
        fprintf(' it measures the wrench that link %s is applying on link %s\n',FTSens.getFirstLinkName(),FTSens.getSecondLinkName());
    end

    % We get the iDynTree:Transform between  the sensor frame and the link frame (sensor_H_link)
    firstLink_H_sensor = iDynTree.Transform();
    secondLink_H_sensor = iDynTree.Transform();

    FTSens.getLinkSensorTransform(FTSens.getFirstLinkIndex(),firstLink_H_sensor);
    FTSens.getLinkSensorTransform(FTSens.getSecondLinkIndex(),secondLink_H_sensor);

    % Get the adjoint transformation as matlab matrix from iDynTree
    % and print it
    firstLink_X_sensor  = firstLink_H_sensor.asAdjointTransform();
    secondLink_X_sensor = secondLink_H_sensor.asAdjointTransform();

    fprintf(' %s_X_sensor is (using linear-angular serialization): \n',FTSens.getFirstLinkName())
    disp(firstLink_X_sensor.toMatlab())
    fprintf(' %s_X_sensor is (using linear-angular serialization): \n',FTSens.getSecondLinkName())
    disp(secondLink_X_sensor.toMatlab())

end
