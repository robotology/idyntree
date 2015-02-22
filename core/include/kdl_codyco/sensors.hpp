/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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

#ifndef IDYNTREE_CORE_SENSORS_HPP
#define IDYNTREE_CORE_SENSORS_HPP

namespace KDL {
    class Wrench;
}


#include <string>

namespace KDL {
namespace CoDyCo {

    enum SensorType
    {
        SIX_AXIS_FORCE_TORQUE = 0,
        NR_OF_SENSOR_TYPES = 1
    };

     /**
     * Virtual interface to Sensor Class.
     * All sensor class inherit from this base class.
     *
     */
    class Sensor {

    public:
        /**
         * Get the id (name) of sensor.
         */
        virtual std::string getName() const = 0;

        /**
         * Get the type of the sensor.
         * @return the type of the sensor.
         */
        virtual SensorType getSensorType() const = 0;

        /**
         * Get the id (name) of the parent entity (Junction or Link).
         */
        virtual std::string getParent() const = 0;

        /**
         * Get the numeric index of the parent of the sensor.
         * Depending on the type of the sensor, the parent could be
         * a Junction or a Link.
         *
         * @return the index of the parent (Junction or Link) of the sensor.
         */
        virtual int getParentIndex() const = 0;

        /**
         * Return true if the sensor has been appropriately configured (all
         * setters where setted, false otherwise.
         *
         */
        virtual bool isValid() const = 0;

        /**
         *  Return a pointer to a copy of this sensor.
         *
         */
        virtual Sensor* clone() const = 0;
    };

    /**
     * Structure representing a group of sensors associated with an UndirectedTree.
     */
    class SensorsTree {
            struct SensorsTreePimpl;
            SensorsTreePimpl * pimpl;

            void constructor(const SensorsTree & other);
            void destructor();
        public:

            /**
             * Constructor.
             */
            SensorsTree();

            /**
             * Copy constructor
             */
            SensorsTree(const SensorsTree& other);

            /**
             * Copy operator
             */
            SensorsTree& operator=(const SensorsTree &other);

            /**
             * Destructor.
             */
            virtual ~SensorsTree();

            /**
             * Add a sensor to the SensorsTree.
             * The sensor index will depend on the order in which the
             * sensor are added to the sensorsTree.
             *
             * The passed sensor will be dynamic casted to the specified sensor type,
             * and will be copied in the sensors tree only if the dynamic cast will be successful.
             *
             * @param[in] sensor_type the SensorType of the added sensor.
             * @param[in] a constant reference to the Sensor to add.
             * @return the sensor index of the newly added sensor, or -1 in case of error.
             */
            int addSensor(const SensorType & sensor_type, const Sensor & sensor);

            /**
             * Get the number of sensors of type sensor_type in this SensorsTree .
             * @return the number of sensors of type sensor_type
             */
            unsigned int getNrOfSensors(const SensorType & sensor_type) const;

            /**
             * Get the index of the sensor
             *
             * @return true if the sensor name is found, false otherwise.
             */
            bool getSensorIndex(const SensorType & sensor_type, const std::string & _sensor_name, unsigned int & sensor_index);

            /**
             * Get the pointer to the sensor of index sensor_index and of type sensor_type
             */
            Sensor * getSensor(const SensorType & sensor_type, int sensor_index) const;

    };

    class SensorsMeasurements
    {
        private:
            struct SensorsMeasurementsPrivateAttributes;
            SensorsMeasurementsPrivateAttributes * pimpl;

        public:
            /**
             * Constructor.
             */
            SensorsMeasurements();

            /**
             * Copy constructor
             */
            SensorsMeasurements(const SensorsMeasurements& other);

            /**
             * Copy operator
             */
            SensorsMeasurements& operator=(const SensorsMeasurements &other);

            /**
             * Destructor.
             */
            virtual ~SensorsMeasurements();

            /**
             * Set the number of sensors of type sensor_type in this SensorsTree .
             * @return true if all went right, false otherwise
             */
            bool setNrOfSensors(const SensorType & sensor_type, unsigned int nrOfSensors);

            /**
             * Get the number of sensors of type sensor_type in this SensorsMeasurements .
             * @return the number of sensors of type sensor_type
             */
            unsigned int getNrOfSensors(const SensorType & sensor_type) const;

            /**
             * Set the wrench measurement for measurement
             *
             * Return true if all is correct (i.e. sensor_index is not out of bounds)
             * and the specified sensor_type uses Wrench as its measurement type.
             */
            bool setMeasurement(const SensorType & sensor_type,
                                const unsigned int & sensor_index,
                                const KDL::Wrench & wrench);
            /**
             * Set the wrench measurement for measurement
             *
             * Return true if all is correct (i.e. sensor_index is not out of bounds)
             * and the specified sensor_type uses Wrench as its measurement type.
             */
            bool getMeasurement(const SensorType & sensor_type,
                                const unsigned int & sensor_index,
                                KDL::Wrench & wrench) const;


    };








}
}



#endif
