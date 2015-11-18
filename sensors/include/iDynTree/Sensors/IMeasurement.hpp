/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Author: Naveen Kuppuswamy
 * email:  naveen.kuppuswamy@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef IDYNTREE_CORE_IMEASUREMENT_HPP
#define IDYNTREE_CORE_IMEASUREMENT_HPP

namespace iDynTree {
    class Wrench;
    class AngularMotionVector3;
    class LinearMotionVector3;
    typedef LinearMotionVector3 LinAcceleration;
    typedef AngularMotionVector3 AngVelocity;
}


namespace iDynTree{
    
    enum MeasurementType
    {
        WRENCH = 0,
        LINEAR_ACCELERATION = 1,
        ANGULAR_VELOCITY = 2,
       // ORIENTATION = 3
    };
    
    
    class IMeasurement{
        
    public :
        IMeasurement();
        ~IMeasurement();
        virtual void  MeasurementType getMeasurementType(void) = 0;       
    };
    
    
    class MeasurementWrench : public IMeasurement, public iDynTree::Wrench
    {
    public:
        MeasurementWrench();
        ~MeasurementWrench();
        MeasurementType getMeasure();
         
        
    };
    
    class MeasurementLinAcceleration : public IMeasurement, public iDynTree::LinAcceleration
    {
    public : 
        MeasurementLinAcceleration();
        ~MeasurementLinAcceleration();
        MeasurementType getMeasurementType();
      //  SensorType getSensorType(void);
    };
    
    class MeasurementAngVelocity : public IMeasurement, public iDynTree::AngVelocity
    {
    public:
        MeasurementAngVelocity();
        ~MeasurementAngVelocity();
        MeasurementType getMeasure
      //  SensorType getSensorType(void);
    };
    
}


#endif