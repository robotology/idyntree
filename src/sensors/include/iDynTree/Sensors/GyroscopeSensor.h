/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#ifndef GYROSCOPE_HPP
#define GYROSCOPE_HPP

#include <iDynTree/Core/GeomVector3.h>

namespace iDynTree
{
    class Transform;
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
        bool setLinkSensorTransform(const iDynTree::Transform & link_H_sensor);

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

        // Documented in LinkSensor
        Transform getLinkSensorTransform() const;

        /**
         * Documented in Sensor
         */
        bool isValid() const;

        /**
         * Documented in Sensor
         */
        Sensor * clone() const;

        /*
         * Documented in Sensor
         */
        bool updateIndices(const Model & model);

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
