
// Bindings specific sensors code
// (to overcome lack of dynamic casts in high level languages)

namespace iDynTree{

%extend SensorsList
{
    // Convert to a dense matrix
    iDynTree::SixAxisForceTorqueSensor * getSixAxisForceTorqueSensor(int sensor_index) const
    {
        iDynTree::SixAxisForceTorqueSensor* p =
            static_cast<iDynTree::SixAxisForceTorqueSensor*>($self->getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,sensor_index));
        return p;
    }
}

}