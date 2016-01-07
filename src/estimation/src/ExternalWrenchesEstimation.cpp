/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
 * website: www.icub.org
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

#include <iDynTree/Estimation/ExternalWrenchesEstimation.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/SubModel.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/ContactWrench.h>

#include <iDynTree/Sensors/Sensors.h>

namespace iDynTree
{

LinkUnknownWrenchContacts::LinkUnknownWrenchContacts(unsigned int nrOfLinks)
{
    this->resize(nrOfLinks);
}


LinkUnknownWrenchContacts::LinkUnknownWrenchContacts(const iDynTree::Model& model)
{
    this->resize(model);
}

void LinkUnknownWrenchContacts::resize(const iDynTree::Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkUnknownWrenchContacts::resize(unsigned int nrOfLinks)
{
    // To avoid dynamic memory allocation at runtime
    // we make sure that the vector have at least spaces for 3 contacts
    const size_t reservedSlots = 3;
    m_linkUnknownWrenchContacts.resize(nrOfLinks);
    for(size_t l=0; l < nrOfLinks; l++)
    {
        m_linkUnknownWrenchContacts[l].resize(0);
        m_linkUnknownWrenchContacts[l].reserve(reservedSlots);
    }
}

const UnknownWrenchContact& LinkUnknownWrenchContacts::contactWrench(const LinkIndex linkIndex, const size_t contactIndex) const
{
    return m_linkUnknownWrenchContacts[linkIndex][contactIndex];
}

UnknownWrenchContact& LinkUnknownWrenchContacts::contactWrench(const LinkIndex linkIndex, const size_t contactIndex)
{
    return m_linkUnknownWrenchContacts[linkIndex][contactIndex];
}

size_t LinkUnknownWrenchContacts::getNrOfContactsForLink(const LinkIndex linkIndex) const
{
    return m_linkUnknownWrenchContacts[linkIndex].size();
}

void LinkUnknownWrenchContacts::setNrOfContactsForLink(const LinkIndex linkIndex, const size_t nrOfContacts)
{
    m_linkUnknownWrenchContacts[linkIndex].resize(nrOfContacts);
    return;
}

void estimateExternalWrenchesBuffers::resize(const SubModelDecomposition& subModels)
{
    this->resize(subModels.getNrOfSubModels());
}

void estimateExternalWrenchesBuffers::resize(const size_t nrOfSubModels)
{
    A.resize(nrOfSubModels);
    x.resize(nrOfSubModels);
    b.resize(nrOfSubModels);
}

bool estimateExternalWrenchesBuffers::isConsistent(const SubModelDecomposition& subModels) const
{
    return (subModels.getNrOfSubModels() == A.size());
}

bool estimateExternalWrenches(const Model& model,
                              const SubModelDecomposition& subModels,
                              const SensorsList& sensors,
                              const LinkUnknownWrenchContacts& unknownWrenches,
                              const LinkVelArray& linkVel,
                              const LinkAccArray& linkProperAcc,
                              const SensorsMeasurements& ftSensorsMeasurements,
                                    estimateExternalWrenchesBuffers& bufs,
                                    LinkContactWrenches& outputContactWrenches)
{

}



}

