/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email: silvio.traversaro@iit.it
 *
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

#ifndef IDYNTREE_ESTIMATION_EXTERNALWRENCHESTIMATION_H
#define IDYNTREE_ESTIMATION_EXTERNALWRENCHESTIMATION_H

#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>

#include <iDynTree/Model/Indeces.h>
#include <iDynTree/Model/LinkState.h>

#include <vector>

namespace iDynTree
{
class Model;
class Traversal;
class SubModelDecomposition;
class LinkContactWrenches;
class SensorsMeasurements;
class SensorsList;
class LinkVelArray;
class LinkAccArray;
class JointPosDoubleArray;
class JointDOFsDoubleArray;


/**
 * Type of a UnknownWrenchContact.
 */
enum UnknownWrenchContactType
{
    /**
     * Contact for which the complete wrench is unknown.
     */
    FULL_WRENCH,

    /**
     * Contact assumed to be a pure force excerted on the contact point.
     */
    PURE_FORCE,

    /**
     * Contact assumed to be a pure force with a known direction excerted on the contact point
     */
    PURE_FORCE_WITH_KNOWN_DIRECTION
};


/**
 * \brief A contact whose wrench is unknown.
 *
 * Class representing a contact for which
 * the approximate center of the contact surface is known,
 * but the wrench is unknown.
 *
 */
struct UnknownWrenchContact
{
    /**
     * Type of the unknown contact.
     */
    UnknownWrenchContactType unknownType;

    /**
     * Position of the center of the contact, in the link frame.
     */
    Position contactPoint;

    /**
     * If unknownType is PURE_FORCE_WITH_KNOWN_DIRECTION,
     * contains the known direction (in link frame) of the force.
     */
    Direction forceDirection;

    /**
     * Unique id identifing the contact.
     * This id is propagated to the contact wrench data structure.
     * It is implemented mainly for compatibility with the skinDynLib library.
     */
    unsigned long contactId;
};

/**
 * A set of UnknownWrenchContact for each link, representing all the contacts
 * between the model and the external environment whose wrench is unkwnon.
 *
 */
class LinkUnknownWrenchContacts
{
private:
    std::vector< std::vector<UnknownWrenchContact> > m_linkUnknownWrenchContacts;

public:
    /**
     * Create a LinkWrenches vector, with the size given
     * by nrOfLinks .
     *
     * @param[in] nrOfLinks the size of the vector.
     */
    LinkUnknownWrenchContacts(unsigned int nrOfLinks = 0);
    LinkUnknownWrenchContacts(const Model & model);

    /**
     * Preserving the number of links, remove all the previously added unknowns.
     *
     */
    void clear();

    /**
     *
     * @param[in]
     */
    void resize(unsigned int nrOfLinks);
    void resize(const Model & model);

    /**
     * Get the number of external contacts for a given link.
     */
    size_t getNrOfContactsForLink(const LinkIndex linkIndex) const;

    /**
     * Set the number of external contacts for a given link.
     */
    void setNrOfContactsForLink(const LinkIndex linkIndex, const size_t nrOfContacts);

    /**
     * Add a new contact for a link.
     */
    void addNewContactForLink(const LinkIndex linkIndex, const UnknownWrenchContact& newContact);

    /**
     * Add a new contact for a frame.
     * If the specified frame is not the a link frame, the method automatically convert the unknown
     * wrench to the relative link frame.
     *
     * @param[in] model the model class for getting frame information.
     * @param[in] frameIndex the index of the frame in which you are expressing the new unknown wrench.
     * @param[in] newContact the new unknown wrench to add.
     * @return true if all went well, false otherwise
     */
    bool addNewContactInFrame(const Model & model,
                              const FrameIndex frameIndex,
                              const UnknownWrenchContact& newContact);

    /**
     * Get a specific ContactWrench
     *
     * @param[in] linkIndex the index of the link for which the contact is retrieved
     * @param[in] contactIndex a index (between 0 and getNrOfContactsForLink(link)-1 ) identifing the specific contact.
     */
    UnknownWrenchContact& contactWrench(const LinkIndex linkIndex, const size_t contactIndex);

    const UnknownWrenchContact& contactWrench(const LinkIndex linkIndex, const size_t contactIndex) const;


    /**
     * Get a human readable description of the LinkUnknownWrenchContacts (for debug)
     */
    std::string toString(const Model & model) const;
};


struct estimateExternalWrenchesBuffers
{
    estimateExternalWrenchesBuffers();
    estimateExternalWrenchesBuffers(const SubModelDecomposition& subModels);
    estimateExternalWrenchesBuffers(const size_t nrOfSubModels, const size_t nrOfLinks);

    /**
     * Resize the struct for the number of submodel
     */
    void resize(const SubModelDecomposition& subModels);
    void resize(const size_t nrOfSubModels, const size_t nrOfLinks);

    size_t getNrOfSubModels() const;
    size_t getNrOfLinks() const;

    /**
     * Check if the buffer size are consistent with the submodel
     * decomposition.
     */
    bool isConsistent(const SubModelDecomposition& subModels) const;

    /**
     * The problem of external wrenches estimation boils down to
     * solve a LS problem in the form argmin_x (Ax-b)^2 .
     */
    std::vector<MatrixDynSize> A;
    std::vector<VectorDynSize> x;
    std::vector<Vector6> b;
    std::vector<MatrixDynSize> pinvA;

    /**
     * We compute the b term for each subtree
     * in a iterative way, so we need a buffer
     * to store it for each link
     */
    LinkWrenches b_contacts_subtree;

    /**
     * We compute the transform between each link
     * and the submodel base, for computing the
     * A matrices
     */
    LinkPositions subModelBase_H_link;
};

/**
 * \brief Estimate the external contact wrenches using the MultiBody Newton-Euler equations.
 *
 * This function is used to estimate the external contacts forces **without** using any measurement
 * of the internal FT sensors. It is tipically used to get data for calibrating the offset of
 * the internal FT sensors.
 */
bool estimateExternalWrenchesWithoutInternalFT(const Model& model,
                                               const Traversal& traversal,
                                               const LinkUnknownWrenchContacts & unknownWrenches,
                                               const JointPosDoubleArray & jointPos,
                                               const LinkVelArray & linkVel,
                                               const LinkAccArray & linkProperAcc,
                                                     estimateExternalWrenchesBuffers & bufs,
                                                     LinkContactWrenches & outputContactWrenches);

/**
 * \brief Estimate the external wrenches trasmitted by the contacts between the model and the external environment.
 *
 * This function exploits the measurements of internal FT sensors (whose structure is contained
 * in the sensors parameters and which measurements are contained in the ftSensorsMeasurements
 * parameters) to compute an estimation of the values of the unknown wrenches specified in the
 * unknownWrenches parameter.
 *
 * @param[in] model the considered model.
 * @param[in] subModels a decomposition of the model along the joint of the six axis F/T sensors.
 * @param[in] sensors a description of the sensors available in the model.
 * @param[in] unknownWrenches a description of the contacts for which the contact wrench is unknown.
 * @param[in] linkVel a vector of link twists, expressed w.r.t to the link orientation and the link origin
 * @param[in] linkProperAcc a vector of link spatial (in the Featherstone sense) and proper accelerations, expressed w.r.t to the link orientation and the link origin
 * @param[in] ftSensorsMeasurements the measurements of the internal six axis F/T sensors.
 * @param[out] outputContactWrenches the estimated contact wrenches.
 * @return true if all went well (the dimension of the inputs are consistent), false otherwise
 *
 */
bool estimateExternalWrenches(const Model& model,
                              const SubModelDecomposition& subModels,
                              const SensorsList& sensors,
                              const LinkUnknownWrenchContacts & unknownWrenches,
                              const JointPosDoubleArray & jointPos,
                              const LinkVelArray & linkVel,
                              const LinkAccArray & linkProperAcc,
                              const SensorsMeasurements & ftSensorsMeasurements,
                                    estimateExternalWrenchesBuffers & bufs,
                                    LinkContactWrenches & outputContactWrenches);

/**
 * \brief Modified forward kinematics for torque/force estimation.
 *
 * This is a version of forward dynamics modified to fit the needs
 * of joint torques/external wrenches estimation.
 *
 * There are several difference with respect to the classical
 * forward kinematics.
 * The first one is that the only inputs necessary related to the base link are
 * the base link classical proper acceleration, the base link angular velocity
 * and the base link angular acceleration. This is because the dynamics
 * of a articulated system does not depend on its linear velocity, and hence
 * the estimation of joint torques/external wrenches is not affected by an offset
 * in linear velocity. This will mean that the link velocitity computed by this
 * algorithm are not the velocity of the links with respect to an inertial frame.
 * Nevertherless they can still be used for estimation.
 *
 * There are two main ways in which the base information is computed: one is
 * exploiting the knoledge that a link is not moving with respect to an inertial frame:
 * in this case the classical proper acceleration boils down to the inverted gravitational
 * acceleration, while the angular velocity and angular accelerations are equal to zero.
 * The other way is to exploit the measure of an accelerometer and of a gyroscope
 * mounted on the base link of the traversal: the accelerometer will then measure
 * directly the classical proper acceleration, while the gyroscope will measure the angular velocity.
 * The angular acceleration can be computed by numerical derivation, or simply neglected if its
 * effect on the estimation is minimal.
 *
 *
 * \param[in] model the input model
 * \param[in] traversal the traversal used to propagate the velocity and the proper acceleration
 * \param[in] base_classicalProperAcc classical proper acceleration of the base origin
 * \param[in] base_angularVel angular velocity of the base link frame
 * \param[in] base_angularAcc angular acceleration of the base link frame
 * \param[in] jointPos joint positions
 * \param[in] jointVel joint velocities
 * \param[in] jointAcc joint accelerations
 * \param[out] linkVel vector of link twists, expressed in the link frame for both orientation and origin
 * \param[out] linkAcc vector of link proper spatial acceleration, expressed in the link frame for both orientation and origin
 *
 */
bool dynamicsEstimationForwardVelAccKinematics(const iDynTree::Model & model,
                                               const iDynTree::Traversal & traversal,
                                               const Vector3 & base_classicalProperAcc,
                                               const Vector3 & base_angularVel,
                                               const Vector3 & base_angularAcc,
                                               const iDynTree::JointPosDoubleArray & jointPos,
                                               const iDynTree::JointDOFsDoubleArray & jointVel,
                                               const iDynTree::JointDOFsDoubleArray & jointAcc,
                                                     iDynTree::LinkVelArray & linkVel,
                                                     iDynTree::LinkAccArray  & linkProperAcc);


}

#endif