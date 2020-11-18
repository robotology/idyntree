/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#ifndef IDYNTREE_SIX_AXIS_FORCE_TORQUE_H
#define IDYNTREE_SIX_AXIS_FORCE_TORQUE_H

namespace iDynTree
{
    class Transform;
    class Wrench;
    class Traversal;
    class Model;
}


#include <iDynTree/Sensors/Sensors.h>

#include <iDynTree/Model/LinkState.h>

#include <vector>

namespace iDynTree {


    /**
     * A six axis force torque sensor class implementation of the Sensor.
     *
     * \ingroup iDynTreeSensors
     */
    class SixAxisForceTorqueSensor: public JointSensor {
    private:
        struct SixAxisForceTorqueSensorPrivateAttributes;
        SixAxisForceTorqueSensorPrivateAttributes * pimpl;

    public:
        /**
         * Constructor.
         */
        SixAxisForceTorqueSensor();

        /**
         * Copy constructor
         */
        SixAxisForceTorqueSensor(const SixAxisForceTorqueSensor& other);

        /**
         * Copy operator
         */
        SixAxisForceTorqueSensor& operator=(const SixAxisForceTorqueSensor &other);

        /**
         * Destructor.
         */
        virtual ~SixAxisForceTorqueSensor();

        /**
         * Set the name (id) of the sensor
         *
         */
        bool setName(const std::string &_name);

        /**
         * Set the transform from the sensor to a first link attached to the sensor.
         *
         * @return true if link_index is one of the two links attached to the FT sensor, false otherwise.
         */
        bool setFirstLinkSensorTransform(const LinkIndex link_index, const iDynTree::Transform & link_H_sensor) const;

        /**
         * Set the transform from the sensor to a the second link attached to the sensor.
         *
         * @return true if link_index is one of the two links attached to the FT sensor, false otherwise.
         */
        bool setSecondLinkSensorTransform(const LinkIndex link_index, const iDynTree::Transform & link_H_sensor) const;

        /**
         * Get the index of the first link attached to the sensor.
         *
         * @return the index of the first link attached to the sensor.
         */
        LinkIndex getFirstLinkIndex() const;

        /**
         * Get the index of the first link attached to the sensor.
         *
         * @return the index of the first link attached to the sensor.
         */
        LinkIndex getSecondLinkIndex() const;

        /**
         * Set the name of the first link at which the FT sensor is attached.
         */
        bool setFirstLinkName(const std::string & name);

        /**
         * Set the name of the first link at which the FT sensor is attached.
         */
        bool setSecondLinkName(const std::string & name);

        /**
         * Get the name of the first link at which the FT sensor is attached.
         */
        std::string getFirstLinkName() const;

        /**
         * Get the name of the second link at which the FT sensor is attached.
         */
        std::string getSecondLinkName() const;

        // Documented in JointSensor
        bool setParentJoint(const std::string &parent);

        // Documented in JointSensor
        bool setParentJointIndex(const JointIndex &parent_index);

        /**
         * The Six Axis Force Torque sensor measure the Force Torque (wrench)
         * applied by a link on another link. This method sets the link
         * on which the measured force is applied.
         * @return the index of the link on which the measure force is applied.
         */
        bool setAppliedWrenchLink(const LinkIndex applied_wrench_index);

        /**
         * Documented in the sensor
         *
         */
        std::string getName() const;

        /**
         * Documented in Sensor
         */
        SensorType getSensorType() const;

        // Documented in JontSensor
        std::string getParentJoint() const;

        // Documented in JointSensor
        JointIndex getParentJointIndex() const;

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
         * The Six Axis Force Torque sensor measure the Force Torque (wrench)
         * applied by a link on another link. This method returns the link
         * on which the measured force is applied.
         * @return the index of the link on which the measure force is applied.
         */
        LinkIndex getAppliedWrenchLink() const;

        /**
         * Check if a given link is attached to this FT sensor.
         * @return true if link_index is attached to the ft sensor, false otherwise
         */
        bool isLinkAttachedToSensor(const LinkIndex link_index) const;


        /**
         * Get the transform from the sensor to the specified link.
         *
         * @return true if link_index is one of the two links attached to the FT sensor, false otherwise.
         */
        bool getLinkSensorTransform(const LinkIndex link_index, iDynTree::Transform & link_H_sensor) const;


        /**
         * Get wrench applied on the specified link expressed in the specified link frame.
         *
         * If the F/T sensors is not connected to link_index, the function will return false
         * and the wrench_applied_on_link will be zeroed.
         *
         * @return true if link_index is one of the two links attached to the FT sensor, false otherwise.
         */
        bool getWrenchAppliedOnLink(const LinkIndex link_index,
                                    const iDynTree::Wrench & measured_wrench,
                                    iDynTree::Wrench & wrench_applied_on_link ) const;

        /**
         * Get the 6x6 matrix that multiplied by the wrench returned by the F/T sensors
         * returnes the wrench applied on the specified link expressed in the specified link frame.
         *
         * If the F/T sensors is not connected to link_index, the function will return false
         * and the matrix will be zeroed.
         *
         * \note This will return and adjoint transformation matrix, possibly with the sign
         *       changed depending on the "direction" of the F/T sensor (i.e. if it is measures
         *       the wrench applied by a link to another, of viceversa).
         *
         * \note The following condition should always hold :
         * ~~~
         * getWrenchAppliedOnLink(link,measured_wrench,wrench_on_link);
         * getWrenchAppliedOnLinkMatrix(link,wrench_applied_on_link_matrix);
         * wrench_applied_on_link_matrix*measured_wrench.toVector() == wrench_on_link
         * ~~~
         *
         * @return true if link_index is one of the two links attached to the FT sensor, false otherwise.
         */
        bool getWrenchAppliedOnLinkMatrix(const LinkIndex link_index,
                                                Matrix6x6 & wrench_applied_on_link_matrix ) const;

        /**
         * Get the 6x6 matrix that multiplied by the wrench applied on the specified link expressed in the specified link frame
         * returns the wrench measured by the F/T sensors.
         *
         * If the F/T sensors is not connected to link_index, the function will return false
         * and the matrix will be zeroed.
         *
         * \note This will return an adjoint transformation matrix, possibly with the sign
         *       changed depending on the "direction" of the F/T sensor (i.e. if it is measures
         *       the wrench applied by a link to another, of viceversa).
         *
         * \note The following condition should always hold :
         * ~~~
         * getWrenchAppliedOnLink(link,measured_wrench,wrench_on_link);
         * getWrenchAppliedOnLinkInverseMatrix(link,wrench_applied_on_link_inverse_matrix);
         * measured_wrench.toVector() == wrench_applied_on_link_inverse_matrix*wrench_on_link
         * ~~~
         *
         * @return true if link_index is one of the two links attached to the FT sensor, false otherwise.
         */
        bool getWrenchAppliedOnLinkInverseMatrix(const LinkIndex link_index,
                                                 Matrix6x6 & wrench_applied_on_link_inverse_matrix ) const;

        /**
         * Predict sensor measurement when given a vector of internal wrenches
         * computed with a given traversal.
         *
         * @return the predicted Measurement
         */
        iDynTree::Wrench predictMeasurement(const Traversal& traversal, const LinkInternalWrenches & intWrenches);

        /**
         *
         */
        std::string toString(const Model & model) const;

    };





}



#endif
