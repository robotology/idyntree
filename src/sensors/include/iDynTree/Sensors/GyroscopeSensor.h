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


#ifndef GYROSCOPE_HPP
#define GYROSCOPE_HPP

namespace iDynTree
{
    class Transform;
    class AngularMotionVector3;
    typedef AngularMotionVector3 AngVelocity;
}

#include <iDynTree/Sensors/Sensors.h>
#include <vector>

namespace iDynTree {

    /**
     * Interface to the Gyroscope class.
     *
     * An implementation of the Gyroscope Sensor
     *
     * \ingroup iDynTreeSensors
     *
     */
    class GyroscopeSensor: public LinkSensor {
    private:
        struct GyroscopePrivateAttributes;
        GyroscopePrivateAttributes * pimpl;

    public:
        /**
         * Constructor.
         */
        GyroscopeSensor();

        /**
         * Copy constructor
         */
        GyroscopeSensor(const GyroscopeSensor& other);

        /**
         * Copy operator
         */
        GyroscopeSensor& operator=(const GyroscopeSensor &other);

        /**
         * Destructor.
         */
        virtual ~GyroscopeSensor();

        /**
         * Set the name (id) of the sensor
         *
         */
        bool setName(const std::string &_name);

        /**
         * Set the transform from the sensor to the parent link sensor is attached to.
         *
         * @return true if link_index is the link attached to the Gyroscope, false otherwise.
         */
        bool setLinkSensorTransform(const iDynTree::Transform & link_H_sensor) const;


        /**
         * Documented in Sensor
         */
        bool setParentLink(const std::string &parent);


        /**
         * Documented in Sensor
         */
        bool setParentLinkIndex(const LinkIndex &parent_index);

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
        std::string getParentLink() const;

        /**
         * Documented in Sensor
         */
        LinkIndex getParentLinkIndex() const;

        /**
         * Documented in Sensor
         */
        bool isValid() const;

        /**
         * Documented in Sensor
         */
        Sensor * clone() const;


        /**
         * Get the transform from the sensor to the parent link.
         *
         * @return Transform associated with the link frame and the sensor
         */
        Transform getLinkSensorTransform(void);

        /**
         * Predict sensor measurement when given the parent link spatial velocity, expressed
         * in the link orientation and wrt the link origin.
         *
         * @return the predicted Measurement of an iDynTree::AngVelocity
         */
        iDynTree::AngVelocity predictMeasurement(const iDynTree::Twist& linkVel);
    };





}



#endif
