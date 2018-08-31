
// Bindings specific sensors code
// (to overcome lack of dynamic casts in high level languages)

namespace iDynTree{

%extend SensorsList
{
    // Expose sensors in a SensorsList

    // TODO implement this as SWIG macro

    iDynTree::SixAxisForceTorqueSensor * getSixAxisForceTorqueSensor(int sensor_index) const
    {
        iDynTree::SixAxisForceTorqueSensor* p =
            static_cast<iDynTree::SixAxisForceTorqueSensor*>($self->getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,sensor_index));
        return p;
    }

    iDynTree::AccelerometerSensor * getAccelerometerSensor(int sensor_index) const
    {
        iDynTree::AccelerometerSensor* p =
            static_cast<iDynTree::AccelerometerSensor*>($self->getSensor(iDynTree::ACCELEROMETER,sensor_index));
        return p;
    }

    iDynTree::GyroscopeSensor * getGyroscopeSensor(int sensor_index) const
    {
        iDynTree::GyroscopeSensor* p =
            static_cast<iDynTree::GyroscopeSensor*>($self->getSensor(iDynTree::GYROSCOPE,sensor_index));
        return p;
    }

    iDynTree::ThreeAxisAngularAccelerometerSensor * getThreeAxisAngularAccelerometerSensor(int sensor_index) const
    {
        iDynTree::ThreeAxisAngularAccelerometerSensor* p =
            static_cast<iDynTree::ThreeAxisAngularAccelerometerSensor*>($self->getSensor(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER,sensor_index));
        return p;
    }

    iDynTree::ThreeAxisForceTorqueContactSensor * getThreeAxisForceTorqueContactSensor(int sensor_index) const
    {
        iDynTree::ThreeAxisForceTorqueContactSensor* p =
            static_cast<iDynTree::ThreeAxisForceTorqueContactSensor*>($self->getSensor(iDynTree::THREE_AXIS_FORCE_TORQUE_CONTACT,sensor_index));
        return p;
    }
}

}
