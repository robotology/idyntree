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


#ifndef ACCELEROMETER_HPP
#define ACCELEROMETER_HPP


namespace iDynTree
{
    class Transform;
    class LinearMotionVector3;
    typedef LinearMotionVector3 LinAcceleration;
    class SpatialAcc;
    class Twist;
}

#include <iDynTree/Sensors/Sensors.hpp>

#include <vector>

namespace iDynTree {

    /**
     * Interface to the Accelerometer class.
     *
     * An implementation of the Accelerometer Sensor
     *
     * \ingroup iDynTreeSensors
     *
     */
    class Accelerometer: public Sensor {
    private:
        struct AccelerometerPrivateAttributes;
        AccelerometerPrivateAttributes * pimpl;

    public:
        /**
         * Constructor.
         */
        Accelerometer();

        /**
         * Copy constructor
         */
        Accelerometer(const Accelerometer& other);

        /**
         * Copy operator
         */
        Accelerometer& operator=(const Accelerometer &other);

        /**
         * Destructor.
         */
        virtual ~Accelerometer();

        /**
         * Set the name (id) of the sensor
         *
         */
        bool setName(const std::string &_name);

       /**
         * Set the transform from the sensor to the parent link attached to the sensor.
         *
         * @return true if link_index is parent link attached to the accelerometer sensor, false otherwise.
         */
        bool setLinkSensorTransform(const iDynTree::Transform & link_H_sensor) const;

        /**
         * Get the index of the parent link attached to the sensor.
         *
         * @return the index of the parent link attached to the sensor.
         */
        int getParentLinkIndex() const;

        /**
         * Set the name of the parent link to which the accelerometer sensor is attached.
         */
        bool setParentLinkName(const std::string & name);

        /**
         * Get the name of the parent link at which the accelerometer sensor is attached.
         */
        std::string getParentLinkName() const;

        /**
         * Documented in Sensor
         */
        bool setParent(const std::string &parent);

        /**
         * Documented in Sensor
         */
        bool setParentIndex(const int &parent_index);
     
        int getLinkIndex() const;
        
        
        
        /**
         * Documented in the sensor
         *
         */
        std::string getName() const;

        /**
         * Documented in Sensor
         */
        SensorType getSensorType() const;


        /**
         * Documented in Sensor
         */
        std::string getParent() const;

        /**
         * Documented in Sensor
         */
        int getParentIndex() const;

        /**
         * Documented in Sensor
         */
        bool isValid() const;

        /**
         * Documented in Sensor
         */
        Sensor * clone() const;


        /**
         * Get the transform from the sensor to the specified link.
         *
         * @return true if link_index is one of the two links attached to the FT sensor, false otherwise.
         */
       bool getLinkSensorTransform(iDynTree::Transform & link_H_sensor) const;


      /**
        * Following method is to be implemented after defining the interface
        * Get wrench applied on the specified link expressed in the specified link frame.
        *
        * @return the predicted measurement as a LinAcceleration
        */
       iDynTree::LinAcceleration predictMeasurement(const iDynTree::SpatialAcc &linkAcc, const iDynTree::Twist &linkTwist);
       
       /**
         * Following method is to be implemented after defining the interface
         * Get wrench applied on the specified link expressed in the specified link frame.
         *
         * @return true if link_index is one of the two links attached to the FT sensor, false otherwise.
         */
   //    bool getAccelerationOfLink(const iDynTree::LinAcceleration & measured_acceleration,
   //                                  iDynTree::LinAcceleration & linear_acceleration_of_link ) const;
    };





}



#endif
