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


#ifndef IDYNTREE_CORE_SENSOR_SIX_AXIS_FT_HPP
#define IDYNTREE_CORE_SENSOR_SIX_AXIS_FT_HPP

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
        bool setFirstLinkSensorTransform(const int link_index, const iDynTree::Transform & link_H_sensor) const;

        /**
         * Set the transform from the sensor to a the second link attached to the sensor.
         *
         * @return true if link_index is one of the two links attached to the FT sensor, false otherwise.
         */
        bool setSecondLinkSensorTransform(const int link_index, const iDynTree::Transform & link_H_sensor) const;

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
        bool setParentJointIndex(const int &parent_index);

        /**
         * The Six Axis Force Torque sensor measure the Force Torque (wrench)
         * applied by a link on another link. This method sets the link
         * on which the measured force is applied.
         * @return the index of the link on which the measure force is applied.
         */
        bool setAppliedWrenchLink(const int applied_wrench_index);

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
        // Deprecated
        bool IDYNTREE_DEPRECATED_WITH_MSG("Use updateIndices() instead") updateIndeces(const Model & model);

        /**
         * The Six Axis Force Torque sensor measure the Force Torque (wrench)
         * applied by a link on another link. This method returns the link
         * on which the measured force is applied.
         * @return the index of the link on which the measure force is applied.
         */
        int getAppliedWrenchLink() const;

        /**
         * Check if a given link is attached to this FT sensor.
         * @return true if link_index is attached to the ft sensor, false otherwise
         */
        bool isLinkAttachedToSensor(const int link_index) const;


        /**
         * Get the transform from the sensor to the specified link.
         *
         * @return true if link_index is one of the two links attached to the FT sensor, false otherwise.
         */
        bool getLinkSensorTransform(const int link_index, iDynTree::Transform & link_H_sensor) const;


        /**
         * Get wrench applied on the specified link expressed in the specified link frame.
         *
         * If the F/T sensors is not connected to link_index, the function will return false
         * and the wrench_applied_on_link will be zeroed.
         *
         * @return true if link_index is one of the two links attached to the FT sensor, false otherwise.
         */
        bool getWrenchAppliedOnLink(const int link_index,
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
