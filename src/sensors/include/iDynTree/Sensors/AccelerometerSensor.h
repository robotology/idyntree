/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#ifndef ACCELEROMETER_HPP
#define ACCELEROMETER_HPP

#include <iDynTree/Core/GeomVector3.h>

namespace iDynTree
{
    class Transform;
    typedef LinearMotionVector3 LinAcceleration;
    class SpatialAcc;
    class Twist;
}

#include <iDynTree/Sensors/Sensors.h>

namespace iDynTree {

    /**
     * Interface to the Accelerometer class.
     *
     * An implementation of the Accelerometer Sensor
     *
     * \ingroup iDynTreeSensors
     *
     */
    class AccelerometerSensor: public LinkSensor
    {
    private:
        struct AccelerometerPrivateAttributes;
        AccelerometerPrivateAttributes * pimpl;

    public:
        /**
         * Constructor.
         */
        AccelerometerSensor();

        /**
         * Copy constructor
         */
        AccelerometerSensor(const AccelerometerSensor& other);

        /**
         * Copy operator
         */
        AccelerometerSensor& operator=(const AccelerometerSensor &other);

        /**
         * Destructor.
         */
        virtual ~AccelerometerSensor();

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
        bool setLinkSensorTransform(const iDynTree::Transform & link_H_sensor);

        /*
         * Documented in Sensor
         */
        bool setParentLink(const std::string &parent);

        /*
         * Documented in Sensor
         */
        bool setParentLinkIndex(const LinkIndex & parent_index);

        /*
         * Documented in the sensor
         *
         */
        std::string getName() const;

        /*
         * Documented in Sensor
         */
        SensorType getSensorType() const;


        /*
         * Documented in Sensor
         */
        std::string getParentLink() const;

        /*
         * Documented in Sensor
         */
        LinkIndex getParentLinkIndex() const;

        // Documented in LinkSensor
        Transform getLinkSensorTransform() const;

        /*
         * Documented in Sensor
         */
        bool isValid() const;

        /*
         * Documented in Sensor
         */
        Sensor * clone() const;

        /*
         * Documented in Sensor
         */
        bool updateIndices(const Model & model);

      /**
        * Following method is to be implemented after defining the interface
        * Get wrench applied on the specified link expressed in the specified link frame.
        *
        * @return the predicted measurement as a LinAcceleration
        */
       iDynTree::LinAcceleration predictMeasurement(const iDynTree::SpatialAcc &linkAcc, const iDynTree::Twist &linkTwist);
    };





}



#endif
