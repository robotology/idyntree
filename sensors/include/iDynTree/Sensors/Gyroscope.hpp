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
}

#include <iDynTree/Sensors/Sensors.hpp>

#include <vector>

namespace iDynTree {

    //class Traversal;

    class Gyroscope: public Sensor {
    private:
        struct GyroscopePrivateAttributes;
        GyroscopePrivateAttributes * pimpl;

    public:
        /**
         * Constructor.
         */
        Gyroscope();

        /**
         * Copy constructor
         */
        Gyroscope(const Gyroscope& other);

        /**
         * Copy operator
         */
        Gyroscope& operator=(const Gyroscope &other);

        /**
         * Destructor.
         */
        virtual ~Gyroscope();

        /**
         * Set the name (id) of the sensor
         *
         */
        bool setName(const std::string &_name);

        /**
         * Set the transform from the sensor to the link attached to the sensor.
         *
         * @return true if link_index is the link attached to the Gyroscope, false otherwise.
         */
        bool setLinkSensorTransform(const int link_index, const iDynTree::Transform & link_H_sensor) const;


        /**
         * Get the index of the link attached to the sensor.
         *
         * @return the index of the link attached to the sensor.
         */
        int getLinkIndex() const;

 

        /**
         * Set the name of the link at which the Gyroscope is attached.
         */
        bool setLinkName(const std::string & name);

        /**
         * Set the name of the first link at which the FT sensor is attached.
         */
//         bool setSecondLinkName(const std::string & name);

        /**
         * Get the name of the link at which the FT sensor is attached.
         */
        std::string getLinkName() const;

        /**
         * Get the name of the second link at which the FT sensor is attached.
         */
//         std::string getSecondLinkName() const;

//         /**
//          * Documented in Sensor
//          */
//         bool setParent(const std::string &parent);
// 
//         /**
//          * Documented in Sensor
//          */
//         bool setParentIndex(const int parent_index);
// 
//         /**
//          * The Six Axis Force Torque sensor measure the Force Torque (wrench)
//          * applied by a link on another link. This method sets the link
//          * on which the measured force is applied.
//          * @return the index of the link on which the measure force is applied.
//          */
// //         bool setAppliedWrenchLink(const int applied_wrench_index);

        /**
         * Documented in the sensor
         *
         */
        std::string getName() const;

        /**
         * Documented in Sensor
         */
        SensorType getSensorType() const;


//         /**
//          * Documented in Sensor
//          */
//         std::string getParent() const;
// 
//         /**
//          * Documented in Sensor
//          */
//         int getParentIndex() const;

        /**
         * Documented in Sensor
         */
        bool isValid() const;

        /**
         * Documented in Sensor
         */
        Sensor * clone() const;

//         /**
//          * The Six Axis Force Torque sensor measure the Force Torque (wrench)
//          * applied by a link on another link. This method returns the link
//          * on which the measured force is applied.
//          * @return the index of the link on which the measure force is applied.
//          */
// //         int getAppliedWrenchLink() const;

        /**
         * Check if a given link is attached to this Gyroscope sensor.
         * @return true if link_index is attached to the Gyroscope sensor, false otherwise
         */
        bool isLinkAttachedToSensor(const int link_index) const;


        /**
         * Get the transform from the sensor to the specified link.
         *
         * @return true if link_index is attached to the FT sensor, false otherwise.
         */
        bool getLinkSensorTransform(const int link_index, iDynTree::Transform & link_H_sensor) const;


        /**
         * Get wrench applied on the specified link expressed in the specified link frame.
         *
         * @return true if link_index is one of the two links attached to the FT sensor, false otherwise.
         */
        bool getAngularVelocityOfLink(const int link_index,
                                    const iDynTree::AngVelocity & measured_angular_velocity,
                                    iDynTree::AngVelocity & angular_velocity_of_link ) const;


    };





}



#endif
