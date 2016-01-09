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
class SubModelDecomposition;
class LinkContactWrenches;
class SensorsMeasurements;
class SensorsList;
class LinkVelArray;
class LinkAccArray;

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
     *
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
     * Get a specific ContactWrench
     *
     * @param[in] linkIndex the index of the link for which the contact is retrieved
     * @param[in] contactIndex a index (between 0 and getNrOfContactsForLink(link)-1 ) identifing the specific contact.
     */
    UnknownWrenchContact& contactWrench(const LinkIndex linkIndex, const size_t contactIndex);

    const UnknownWrenchContact& contactWrench(const LinkIndex linkIndex, const size_t contactIndex) const;
};


struct estimateExternalWrenchesBuffers
{
    /**
     * Resize the struct for the number of submodel
     */
    void resize(const SubModelDecomposition& subModels);
    void resize(const size_t nrOfSubModels, const size_t nrOfLinks);

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
 * Estimate the external wrenches trasmitted by the contacts between the model and the external environment.
 *
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
                              const LinkVelArray & linkVel,
                              const LinkAccArray & linkProperAcc,
                              const SensorsMeasurements & ftSensorsMeasurements,
                                    estimateExternalWrenchesBuffers & bufs,
                                    LinkContactWrenches & outputContactWrenches);


}

#endif


