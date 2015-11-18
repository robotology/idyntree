# include "iDynTree/Core/Wrench.h"
# include "iDynTree/Core/AngularMotionVector3.h"
# include "iDynTree/Core/LinearMotionVector3.h"

# include "iDynTree/Sensors/IMeasurement.hpp"

// #include "iDynTree/Sensors/Sensors.hpp"

namespace iDynTree{
///////////////////////////////////////////////////////////////////////////////
///// IMeasurement
///////////////////////////////////////////////////////////////////////////////
IMeasurement::IMeasurement()
{

}
IMeasurement::~IMeasurement()
{

}

///////////////////////////////////////////////////////////////////////////////
///// MeasurementWrench
///////////////////////////////////////////////////////////////////////////////
MeasurementWrench::MeasurementWrench()
{

}
MeasurementWrench::~MeasurementWrench()
{

}
MeasurementType MeasurementWrench::getMeasure()
{
    return(WRENCH);
}

///////////////////////////////////////////////////////////////////////////////
///// MeasurementLinAcceleration
///////////////////////////////////////////////////////////////////////////////
MeasurementType MeasurementLinAcceleration::getMeasurementType()
{
    return(LINEAR_ACCELERATION);
}


MeasurementLinAcceleration::~MeasurementLinAcceleration()
{

}
MeasurementLinAcceleration::MeasurementLinAcceleration()
{

}

///////////////////////////////////////////////////////////////////////////////
///// MeasurementAngVelocity
///////////////////////////////////////////////////////////////////////////////

MeasurementAngVelocity::MeasurementAngVelocity()
{

}
MeasurementAngVelocity::~MeasurementAngVelocity()
{

}


MeasurementType MeasurementAngVelocity::getMeasurementType()
{       
    return(ANGULAR_VELOCITY);
    
}

