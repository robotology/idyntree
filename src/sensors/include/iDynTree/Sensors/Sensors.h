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



namespace iDynTree {
    class Wrench;
    class AngularMotionVector3;
    class LinearMotionVector3;
    typedef LinearMotionVector3 LinAcceleration;
    typedef AngularMotionVector3 AngVelocity;
}

#include <string>
#include <vector>
#include <iterator>

#include <iDynTree/Core/VectorDynSize.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Indeces.h>

namespace iDynTree {

    enum SensorType
    {
        SIX_AXIS_FORCE_TORQUE = 0,
        ACCELEROMETER = 1,
        GYROSCOPE = 2
    };

    // This should be equal to the number of option
    //  in the SensorType enum
    const int NR_OF_SENSOR_TYPES = 3;

    /**
     * Short function to check if a SensorType is a LinkSensor
     */
    inline bool isLinkSensor(const SensorType type)
    {
        switch(type)
        {
        case SIX_AXIS_FORCE_TORQUE:
            return false;
        case ACCELEROMETER:
        case GYROSCOPE:
            return true;
        }
    }

    /**
     * Short function to check if a SensorType is
     */
    inline bool isJointSensor(const SensorType type)
    {
        return !isLinkSensor(type);
    }

    inline unsigned int getSensorTypeSize(const SensorType type)
    {
        switch(type)
        {
            case SIX_AXIS_FORCE_TORQUE:
                return 6;
            case ACCELEROMETER:
            case GYROSCOPE:
                return 3;
            default:
                return 0;
        }
    }


    /**
     * Interface for Sensor classes in iDynTree .
     *
     * All sensor classes inherit from this base class.
     *
     * \ingroup iDynTreeSensors
     *
     */
    class Sensor
    {
    public:
        /**
         * Virtual destructor
         */
        virtual ~Sensor() = 0;

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
         * Return true if the sensor has been appropriately configured (all
         * setters where setted, false otherwise.
         *
         */
        virtual bool isValid() const = 0;

        /**
         * Set the id (name) of sensor.
         */
        virtual bool setName(const std::string &) = 0;

        /**
         *  Return a pointer to a copy of this sensor.
         *
         */
        virtual Sensor* clone() const = 0;

        /**
         * Update all the indeces (link/frames) contained in this sensor.
         */
        virtual bool updateIndeces(const Model & model) = 0;
    };

    /**
     * Interface for Sensor that are associated to a Joint.
     *
     * All joint sensor classes inherit from this base class.
     *
     * \ingroup iDynTreeSensors
     *
     */
    class JointSensor: public Sensor
    {
    public:
        /**
         * Virtual destructor
         */
        virtual ~JointSensor() = 0;

        /**
         * Get the name of the parent Joint.
         */
        virtual std::string getParentJoint() const = 0;

        /**
         * Get the numeric index of the parent of the sensor.
         * Depending on the type of the sensor, the parent could be
         * a Junction or a Link.
         *
         * @return the index of the parent (Junction or Link) of the sensor.
         */
        virtual int getParentJointIndex() const = 0;

        /**
         * Set the name of the parent Joint.
         */
        virtual bool setParentJoint(const std::string & parentJointName) = 0;

        /**
         * Set the numeric index of the parent joint of the sensor.
         */
        virtual bool setParentJointIndex(const int &) = 0;
    };

    /**
     * Interface for Sensor that are associated to a Link.
     *
     * All link sensor classes inherit from this base class.
     *
     * \ingroup iDynTreeSensors
     *
     */
    class LinkSensor: public Sensor
    {
    public:
        /**
         * Virtual destructor
         */
        virtual ~LinkSensor() = 0;

        /**
         * Get the name of the parent Link.
         */
        virtual std::string getParentLink() const = 0;

        /**
         * Get the numeric index of the parent Link.
         *
         * @return the index of the parent Link of the sensor.
         */
        virtual LinkIndex getParentLinkIndex() const = 0;

        /**
         * Return the transform that applied on a element
         * expressed in sensor frames it transform it
         * in one expressed in the link frame (\f[ {}^l H_s \f]).
         *
         * @return the link_H_sensor transform
         */
        virtual Transform getLinkSensorTransform() const = 0;


        /**
         * Set the name of the parent Link.
         */
        virtual bool setParentLink(const std::string & parentLinkName) = 0;

        /**
         * Set the numeric index of the parent joint of the sensor.
         */
        virtual bool setParentLinkIndex(const LinkIndex &) = 0;
    };

    /**
     * Structure representing a group of sensors associated with an UndirectedTree.
     *
     * \ingroup iDynTreeSensors
     */
    class SensorsList {
            struct SensorsListPimpl;
            SensorsListPimpl * pimpl;

            void constructor(const SensorsList & other);
            void destructor();
        public:

            /// Iterators for the SensorsList class
            class TypedIterator;
            class ConstTypedIterator;
            typedef TypedIterator typed_iterator;
            typedef ConstTypedIterator const_typed_iterator;
            class Iterator;
            class ConstIterator;
            typedef Iterator iterator;
            typedef ConstIterator const_iterator;

            /**
             * Constructor.
             */
            SensorsList();

            /**
             * Copy constructor
             */
            SensorsList(const SensorsList& other);

            /**
             * Copy operator
             */
            SensorsList& operator=(const SensorsList &other);

            /**
             * Destructor.
             */
            virtual ~SensorsList();

            /**
             * \brief Add a sensor to the SensorsTree.
             *
             * The initial sensor index will depend on the order in which the
             * sensor are added to the sensorsTree.
             * The sensor index can then be changed with a call to the setSerialization method.
             *
             * The passed sensor will be dynamic casted to the specified sensor type,
             * and will be copied in the sensors tree only if the dynamic cast will be successful.
             *
             * @param[in] a constant reference to the Sensor to add.
             * @return the sensor index of the newly added sensor, or -1 in case of error.
             */
            int addSensor(const Sensor & sensor);

            /**
             * Change the serialization of a specific sensor type.
             */
            bool setSerialization(const SensorType & sensor_type, const std::vector<std::string> & serializaton);

            /**
             * Get the serialization of a specific sensor type.
             */
            bool getSerialization(const SensorType & sensor_type, std::vector<std::string> & serializaton);

            /**
             * Get the number of sensors of type sensor_type in this SensorsList .
             * @return the number of sensors of type sensor_type
             */
            unsigned int getNrOfSensors(const SensorType & sensor_type) const;

            /**
             * Get the index of a sensor of type sensor_type in this SensorList
             *
             * @return true if the sensor name is found, false otherwise.
             */
            bool getSensorIndex(const SensorType & sensor_type, const std::string & _sensor_name, unsigned int & sensor_index) const;

            /**
             * Get the index of a sensor of type sensor_type and with name sensor_name
             *
             * @return the sensor index if the sensor_name is found, -1 otherwise.
             *
             * \note Some languages do not support well in-output parameters, so we provided this
             *       method as an alternative to the three-arguments getSensorIndex
             */
            int getSensorIndex(const SensorType & sensor_type, const std::string & _sensor_name) const;

            /**
             * Get the total size of sensor measurements.
             */
            size_t getSizeOfAllSensorsMeasurements() const;


            /**
             * Get the pointer to the sensor of index sensor_index and of type sensor_type
             *
             * \return the pointer of sensor, of 0 if sensor_index is out of bounds
             */
            Sensor * getSensor(const SensorType & sensor_type, int sensor_index) const;

        bool removeSensor(const SensorType & sensor_type, const std::string & _sensor_name);
        bool removeSensor(const SensorType & sensor_type, const unsigned int sensor_index);
        bool removeAllSensorsOfType(const SensorType & sensor_type);

        iterator allSensorsIterator();
        const_iterator allSensorsIterator() const;
        typed_iterator sensorsIteratorForType(const iDynTree::SensorType &sensor_type);
        const_typed_iterator sensorsIteratorForType(const iDynTree::SensorType &sensor_type) const;

    };

    class SensorsList::TypedIterator
    {
    private:
        TypedIterator(std::vector<Sensor *>& list);
        friend class SensorsList;

        std::vector<Sensor *>& iteratingList;
        std::vector<Sensor *>::iterator internalIterator;

    public:
        typedef std::ptrdiff_t difference_type;
        typedef Sensor* value_type;
        typedef value_type& reference;
        typedef value_type* pointer;
        typedef std::input_iterator_tag iterator_category;

        TypedIterator& operator++();
        TypedIterator operator++(int);

        bool operator==(const TypedIterator&) const;
        bool operator==(const ConstTypedIterator&) const;
        inline bool operator!=(const TypedIterator& s) const { return !this->operator==(s); }
        inline bool operator!=(const ConstTypedIterator& s) const { return !this->operator==(s); }

        reference operator*() const;
        pointer operator->() const;

        bool isValid() const;
    };

    class SensorsList::ConstTypedIterator
    {
    private:
        ConstTypedIterator(std::vector<Sensor *>& list);
        friend class SensorsList;

        std::vector<Sensor *>& iteratingList;
        std::vector<Sensor *>::const_iterator internalIterator;

        void constructor();

    public:
        typedef std::ptrdiff_t difference_type;
        typedef Sensor* value_type;
        typedef const value_type& reference;
        typedef const value_type* pointer; //Not sure. Maybe this should be simply Sensor*??
        typedef std::input_iterator_tag iterator_category;

        ConstTypedIterator(const TypedIterator&);

        ConstTypedIterator& operator++();
        ConstTypedIterator operator++(int);

        bool operator==(const ConstTypedIterator&) const;
        bool operator==(const TypedIterator&) const;
        inline bool operator!=(const ConstTypedIterator& s) const { return !this->operator==(s); }
        inline bool operator!=(const TypedIterator& s) const { return !this->operator==(s); }

        reference operator*() const;
        pointer operator->() const;
        
        bool isValid() const;
    };

    class SensorsList::Iterator
    {
    private:
        //it can be created only by SensorsList class
        Iterator(std::vector< std::vector<Sensor *> >&);
        friend class SensorsList;

        std::vector<Sensor *>::iterator internalIterator;
        std::vector< std::vector<Sensor *> >::iterator externalIterator;
        std::vector< std::vector<Sensor *> >& iteratingList;

    public:
        typedef std::ptrdiff_t difference_type;
        typedef Sensor* value_type;
        typedef value_type& reference;
        typedef value_type* pointer; //Not sure. Maybe this should be simply Sensor*??
        typedef std::input_iterator_tag iterator_category;

        Iterator& operator++();
        Iterator operator++(int);

        bool operator==(const Iterator&) const;
        bool operator==(const ConstIterator&) const;
        inline bool operator!=(const Iterator& s) const { return !this->operator==(s); }
        inline bool operator!=(const ConstIterator& s) const { return !this->operator==(s); }

        reference operator*() const;
        pointer operator->() const;
        
        bool isValid() const;
    };

    class SensorsList::ConstIterator
    {
    private:
        //it can be created only by SensorsList class
        ConstIterator(std::vector< std::vector<Sensor *> >&);
        friend class SensorsList;

        std::vector<Sensor *>::const_iterator internalIterator;
        std::vector< std::vector<Sensor *> >::const_iterator externalIterator;
        std::vector< std::vector<Sensor *> >& iteratingList;

        void constructor();

    public:
        typedef std::ptrdiff_t difference_type;
        typedef Sensor* value_type;
        typedef const value_type& reference;
        typedef const value_type* pointer; //Not sure. Maybe this should be simply Sensor*??
        typedef std::input_iterator_tag iterator_category;

        ConstIterator(const Iterator&);

        ConstIterator& operator++();
        ConstIterator operator++(int);

        bool operator==(const ConstIterator&) const;
        bool operator==(const Iterator&) const;
        inline bool operator!=(const ConstIterator& s) const { return !this->operator==(s); }
        inline bool operator!=(const Iterator& s) const { return !this->operator==(s); }

        reference operator*() const;
        pointer operator->() const;

        bool isValid() const;
    };


    /**
     * A list of measurements associated with a SensorsList .
     *
     * \ingroup iDynTreeSensors
     */
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
             * Constructor from SensorList
             */
            SensorsMeasurements(const SensorsList &sensorList);

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
             * Resize and reset the measurement vectors
             * @return true if all went right, false otherwise
             */
            bool resize(const SensorsList & sensorsList);

            /**
             * Returns a double vector of all the sensors measurements
             * @return true if all went right, false otherwise
             */
            bool toVector(VectorDynSize & measurementVector) const;

            /**
             * Set the measurement for the specified sensor
             *
             * Return true if all is correct (i.e. sensor_index is not out of bounds)
             * and the specified sensor_type uses Wrench as its measurement type.
             */
            bool setMeasurement(const SensorType & sensor_type,
                                const unsigned int & sensor_index,
                                const iDynTree::Wrench & measurement);

            bool setMeasurement(const SensorType & sensor_type,
                                const unsigned int & sensor_index,
                                const iDynTree::LinAcceleration & measurement);

            bool setMeasurement(const SensorType & sensor_type,
                                const unsigned int & sensor_index,
                                const iDynTree::AngVelocity & measurement);


            /**
             * Get the measurement for a specified sensor
             *
             * Return true if all is correct (i.e. sensor_index is not out of bounds)
             * and the specified sensor_type uses its appropriate type as its measurement type.
             */
            bool getMeasurement(const SensorType & sensor_type,
                                const unsigned int & sensor_index,
                                iDynTree::Wrench & measurement) const;
            bool getMeasurement(const SensorType & sensor_type,
                                const unsigned int & sensor_index,
                                iDynTree::LinAcceleration &measurement) const;
            bool getMeasurement(const SensorType & sensor_type,
                                const unsigned int & sensor_index,
                                iDynTree::AngVelocity &measurement) const;

            /**
             * Get the total size of sensor measurements.
             *
             * \note this is the size of vector returned by toVector.
             */
            size_t getSizeOfAllSensorsMeasurements() const;
    };

}



#endif
