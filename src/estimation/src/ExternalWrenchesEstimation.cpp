// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
#include <iDynTree/ExternalWrenchesEstimation.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/EigenMathHelpers.h>
#include <iDynTree/SpatialMomentum.h>

#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/SubModel.h>
#include <iDynTree/LinkState.h>
#include <iDynTree/JointState.h>
#include <iDynTree/ContactWrench.h>
#include <iDynTree/LinkTraversalsCache.h>

#include <iDynTree/Sensors.h>
#include <iDynTree/SixAxisForceTorqueSensor.h>

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

void LinkUnknownWrenchContacts::clear()
{
    // To avoid dynamic memory allocation at runtime
    // we make sure that the vector have at least spaces for 3 contacts
    const size_t reservedSlots = 3;

    for(size_t l=0; l < m_linkUnknownWrenchContacts.size(); l++)
    {
        m_linkUnknownWrenchContacts[l].resize(0);
        m_linkUnknownWrenchContacts[l].reserve(reservedSlots);
    }
}


void LinkUnknownWrenchContacts::resize(const iDynTree::Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkUnknownWrenchContacts::resize(unsigned int nrOfLinks)
{
    m_linkUnknownWrenchContacts.resize(nrOfLinks);
    clear();
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

void LinkUnknownWrenchContacts::addNewContactForLink(const LinkIndex linkIndex, const UnknownWrenchContact& newContact)
{
    m_linkUnknownWrenchContacts[linkIndex].push_back(newContact);
}

bool LinkUnknownWrenchContacts::addNewContactInFrame(const Model & model,
                                                     const FrameIndex frameIndex,
                                                     const UnknownWrenchContact& unknownContactInFrame)
{
    if( !model.isValidFrameIndex(frameIndex) )
    {
        std::stringstream err;
        err << "Unknown frame index " << frameIndex << " in model that has " << model.getNrOfFrames() << " frames.";
        reportError("LinkUnknownWrenchContacts","addNewContactInFrame",err.str().c_str());
        return false;
    }

    // Get link_H_frame transform
    iDynTree::Transform link_H_frame = model.getFrameTransform(frameIndex);

    // Get the link of the frame
    LinkIndex linkIndex = model.getFrameLink(frameIndex);

    if( !model.isValidLinkIndex(linkIndex) )
    {
        std::stringstream err;
        err << "Unknown link index " << linkIndex << " in model that has " << model.getNrOfLinks() << " links.";
        reportError("LinkUnknownWrenchContacts","addNewContactInFrame",err.str().c_str());
        return false;
    }

    UnknownWrenchContact unknownContactInLink;

    unknownContactInLink.unknownType = unknownContactInFrame.unknownType;
    unknownContactInLink.contactPoint = link_H_frame*unknownContactInFrame.contactPoint;

    if( unknownContactInFrame.unknownType == PURE_FORCE_WITH_KNOWN_DIRECTION )
    {
        unknownContactInLink.forceDirection =  link_H_frame*unknownContactInFrame.forceDirection;
    }

    if (unknownContactInFrame.unknownType == NO_UNKNOWNS)
    {
        unknownContactInLink.knownWrench =  link_H_frame*unknownContactInFrame.knownWrench;
    }

    addNewContactForLink(linkIndex,unknownContactInLink);

    return true;
}

bool LinkUnknownWrenchContacts::addNewUnknownFullWrenchInFrameOrigin(const Model& model, const FrameIndex frame)
{
    return this->addNewContactInFrame(model,frame,UnknownWrenchContact(FULL_WRENCH,Position::Zero()));
}




std::string LinkUnknownWrenchContacts::toString(const Model& model) const
{
    std::stringstream ss;

    size_t nrOfLinks = m_linkUnknownWrenchContacts.size();
    for(size_t l=0; l < nrOfLinks; l++)
    {
        size_t nrOfContacts = this->getNrOfContactsForLink(l);

        if( nrOfContacts > 0 )
        {
            ss << "Unknown contacts on link " << model.getLinkName(l) << ":" << std::endl;
            for(size_t c=0; c < nrOfContacts; c++ )
            {
                switch(this->contactWrench(l,c).unknownType)
                {
                    case FULL_WRENCH:
                        ss << "One full wrench contact with pos: " << this->contactWrench(l,c).contactPoint.toString() << ":" << std::endl;
                        break;
                    case PURE_FORCE:
                        ss << "One pure force contact with pos: " << this->contactWrench(l,c).contactPoint.toString() << ":" << std::endl;
                        break;
                    case PURE_FORCE_WITH_KNOWN_DIRECTION:
                        ss << "One pure force contact with known direction with pos: " << this->contactWrench(l,c).contactPoint.toString() << ":" << std::endl;
                        break;
                    case NO_UNKNOWNS:
                        ss << "One fully known contact wrench with value: " << this->contactWrench(l,c).knownWrench.toString() << ":" << std::endl;
                        break;
                }
            }
        }
    }
    return ss.str();
}

estimateExternalWrenchesBuffers::estimateExternalWrenchesBuffers()
{
    resize(0,0);
}


estimateExternalWrenchesBuffers::estimateExternalWrenchesBuffers(const SubModelDecomposition& subModels)
{
    resize(subModels);
}

estimateExternalWrenchesBuffers::estimateExternalWrenchesBuffers(const size_t nrOfSubModels, const size_t nrOfLinks)
{
    resize(nrOfSubModels,nrOfLinks);
}


void estimateExternalWrenchesBuffers::resize(const SubModelDecomposition& subModels)
{
    this->resize(subModels.getNrOfSubModels(),subModels.getNrOfLinks());
}

void estimateExternalWrenchesBuffers::resize(const size_t nrOfSubModels, const size_t nrOfLinks)
{
    A.resize(nrOfSubModels);
    x.resize(nrOfSubModels);
    b.resize(nrOfSubModels);

    b_contacts_subtree.resize(nrOfLinks);

    subModelBase_H_link.resize(nrOfLinks);
}

size_t estimateExternalWrenchesBuffers::getNrOfLinks() const
{
    return b_contacts_subtree.getNrOfLinks();
}

size_t estimateExternalWrenchesBuffers::getNrOfSubModels() const
{
    return A.size();
}

bool estimateExternalWrenchesBuffers::isConsistent(const SubModelDecomposition& subModels) const
{
    return (subModels.getNrOfSubModels() == A.size()) &&
           (b_contacts_subtree.getNrOfLinks() == subModels.getNrOfLinks());
}

/**
 * \brief return the measured wrench applied on a given link
 *
 * Get the sum of all the wrenches applied to a given link
 * by neighbors joints which are equipped with 6 axis F/T sensors.
 *
 * The resulting wrench is the one applied **on** the link
 * and expressed in the link frame (both origin and orientation).
 *
 * @param[in] sensor_list list of considered sensors
 * @param[in] sensor_measures list of measurements of the considered sensors
 * @param[in] link_id the index of the considered link.
 */
Wrench getMeasuredWrench(const SensorsList & sensor_list,
                         const SensorsMeasurements & sensor_measures,
                         const LinkIndex link_id)
{
    size_t NrOfFTSensors = sensor_list.getNrOfSensors(SIX_AXIS_FORCE_TORQUE);
    ::iDynTree::Wrench total_measured_applied_wrench = ::iDynTree::Wrench::Zero();
    for(size_t ft=0; ft < NrOfFTSensors; ft++ )
    {
        ::iDynTree::SixAxisForceTorqueSensor * sens
            = (::iDynTree::SixAxisForceTorqueSensor *) sensor_list.getSensor(::iDynTree::SIX_AXIS_FORCE_TORQUE,ft);

        assert(sens != 0);

        Wrench measured_wrench_on_link = Wrench::Zero();
        Wrench measured_wrench_by_sensor;

        bool ok = sensor_measures.getMeasurement(SIX_AXIS_FORCE_TORQUE,ft,measured_wrench_by_sensor);

        IDYNTREE_UNUSED(ok);
        assert(ok);

        // If the sensor with index ft is not attached to the link
        // this function return a zero wrench
        sens->getWrenchAppliedOnLink(link_id,measured_wrench_by_sensor,measured_wrench_on_link);

        //Sum the given wrench to the return value
        total_measured_applied_wrench = total_measured_applied_wrench+measured_wrench_on_link;
    }

    return total_measured_applied_wrench;
}

Wrench computeKnownTermsOfEstimationEquationWithoutInternalFT(const Model& model,
                                                              const Traversal& modelTraversal,
                                                              const JointPosDoubleArray & jointPos,
                                                              const LinkVelArray& linkVel,
                                                              const LinkAccArray& linkProperAcc,
                                                                    estimateExternalWrenchesBuffers& bufs)
{
    // First compute the known term of the estimation for each link:
    // this loop is similar to the dynamic phase of the RNEA
    // \todo pimp up performance as done in RNEADynamicPhase
     for(int traversalEl = modelTraversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
     {
         LinkConstPtr visitedLink = modelTraversal.getLink(traversalEl);
         LinkIndex    visitedLinkIndex = visitedLink->getIndex();
         LinkConstPtr parentLink  = modelTraversal.getParentLink(traversalEl);

         const iDynTree::SpatialInertia & I = visitedLink->getInertia();
         const iDynTree::SpatialAcc     & properAcc = linkProperAcc(visitedLinkIndex);
         const iDynTree::Twist          & v = linkVel(visitedLinkIndex);
         bufs.b_contacts_subtree(visitedLinkIndex) = I*properAcc + v*(I*v);

         // Iterate on childs of visitedLink
         // We obtain all the children as all the neighbors of the link, except
         // for its parent
         // \todo TODO this point is definitly Tree-specific
         // \todo TODO this "get child" for is duplicated in the code, we
         //            should try to consolidate it
         for(unsigned int neigh_i=0; neigh_i < model.getNrOfNeighbors(visitedLinkIndex); neigh_i++)
         {
             LinkIndex neighborIndex = model.getNeighbor(visitedLinkIndex,neigh_i).neighborLink;
             if( !parentLink || neighborIndex != parentLink->getIndex() )
             {
                 LinkIndex childIndex = neighborIndex;
                 IJointConstPtr neighborJoint = model.getJoint(model.getNeighbor(visitedLinkIndex,neigh_i).neighborJoint);
                 Transform visitedLink_X_child = neighborJoint->getTransform(jointPos,visitedLinkIndex,childIndex);

                 // One term of the sum in Equation 5.20 in Featherstone 2008
                 bufs.b_contacts_subtree(visitedLinkIndex) = bufs.b_contacts_subtree(visitedLinkIndex)
                                                            + visitedLink_X_child*bufs.b_contacts_subtree(childIndex);
             }
         }

         if( parentLink == 0 )
         {
             // If the visited link is the base of the submodel, the
             // computed known terms is the known term of the submodel itself
             return bufs.b_contacts_subtree(visitedLinkIndex);
         }
     }

     assert(false);
     return Wrench::Zero();
}

Wrench computeKnownTermsOfEstimationEquationWithInternalFT(const Model& model,
                                                           const Traversal& subModelTraversal,
                                                           const SensorsList& sensors,
                                                           const JointPosDoubleArray & jointPos,
                                                           const LinkVelArray& linkVel,
                                                           const LinkAccArray& linkProperAcc,
                                                           const SensorsMeasurements& ftSensorsMeasurements,
                                                                 estimateExternalWrenchesBuffers& bufs)
{
    // First compute the known term of the estimation for each link:
    // this loop is similar to the dynamic phase of the RNEA
    // \todo pimp up performance as done in RNEADynamicPhase
     for(int traversalEl = subModelTraversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
     {
         LinkConstPtr visitedLink = subModelTraversal.getLink(traversalEl);
         LinkIndex    visitedLinkIndex = visitedLink->getIndex();
         LinkConstPtr parentLink  = subModelTraversal.getParentLink(traversalEl);

         const iDynTree::SpatialInertia & I = visitedLink->getInertia();
         const iDynTree::SpatialAcc     & properAcc = linkProperAcc(visitedLinkIndex);
         const iDynTree::Twist          & v = linkVel(visitedLinkIndex);
         bufs.b_contacts_subtree(visitedLinkIndex) = I*properAcc + v*(I*v) - getMeasuredWrench(sensors,ftSensorsMeasurements,visitedLinkIndex);

         // Iterate on childs of visitedLink
         // We obtain all the children as all the neighbors of the link, except
         // for its parent
         // \todo TODO this point is definitly Tree-specific
         // \todo TODO this "get child" for is duplicated in the code, we
         //            should try to consolidate it
         for(unsigned int neigh_i=0; neigh_i < model.getNrOfNeighbors(visitedLinkIndex); neigh_i++)
         {
             LinkIndex neighborIndex = model.getNeighbor(visitedLinkIndex,neigh_i).neighborLink;
             // Check if this neighbor is a child of the link according to this traversal
             if( subModelTraversal.isParentOf(visitedLinkIndex,neighborIndex) )
             {
                 LinkIndex childIndex = neighborIndex;
                 IJointConstPtr neighborJoint = model.getJoint(model.getNeighbor(visitedLinkIndex,neigh_i).neighborJoint);
                 Transform visitedLink_X_child = neighborJoint->getTransform(jointPos,visitedLinkIndex,childIndex);

                 // One term of the sum in Equation 5.20 in Featherstone 2008
                 bufs.b_contacts_subtree(visitedLinkIndex) = bufs.b_contacts_subtree(visitedLinkIndex)
                                                            + visitedLink_X_child*bufs.b_contacts_subtree(childIndex);
             }
         }

         if( parentLink == 0 )
         {
             // If the visited link is the base of the submodel, the
             // computed known terms is the known term of the submodel itself
             return bufs.b_contacts_subtree(visitedLinkIndex);
         }
     }

     // If we reach this point of the code, something is really really wrong
     assert(false);
     return Wrench::Zero();
}

size_t countUnknowns(const Traversal& traversal, const LinkUnknownWrenchContacts& unknownWrenches)
{
    size_t unknowns = 0;

    for(int traversalEl = traversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex    visitedLinkIndex = visitedLink->getIndex();

        // While we are in this loop, we also keep track of the number of unknowns
        for(size_t contact = 0;
            contact < unknownWrenches.getNrOfContactsForLink(visitedLinkIndex);
            contact++ )
        {
            const UnknownWrenchContact & unknownWrench = unknownWrenches.contactWrench(visitedLinkIndex,contact);

            switch( unknownWrench.unknownType )
            {
                case FULL_WRENCH:
                    unknowns += 6;
                    break;
                case PURE_FORCE:
                    unknowns += 3;
                    break;
                case PURE_FORCE_WITH_KNOWN_DIRECTION:
                    unknowns += 1;
                    break;
                case NO_UNKNOWNS:
                    //unknowns += 0;
                    break;
                default:
                    assert(false);
                    break;
            }
        }
    }

    return unknowns;
}

void computeMatrixOfEstimationEquationAndExtWrenchKnownTerms(const Model& model,
                                                             const Traversal& traversal,
                                                             const LinkUnknownWrenchContacts& unknownWrenches,
                                                             const JointPosDoubleArray & jointPos,
                                                             const size_t subModelIndex,
                                                                   estimateExternalWrenchesBuffers& bufs)
{
    // Count unknowns
     size_t unknowns = countUnknowns(traversal,unknownWrenches);

     // Now we compute the A matrix
     assert(unknowns > 0);
     bufs.A[subModelIndex].resize(6,unknowns);
     bufs.x[subModelIndex].resize(unknowns);

     // As a first step, we need to compute the transform between each link and the base
     // of its submodel (we are computing the estimation equation in the submodel base frame
     computeTransformToTraversalBase(model,traversal,jointPos,bufs.subModelBase_H_link);

     Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Aeig = toEigen(bufs.A[subModelIndex]);
     int AcolsToWrite = 0;
     // We then need to compute the different submatrix of A for each type of unknown
     for(int traversalEl = (int)traversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
     {
         LinkConstPtr visitedLink = traversal.getLink(traversalEl);
         LinkIndex    visitedLinkIndex = visitedLink->getIndex();

         size_t nrOfContactForLink = unknownWrenches.getNrOfContactsForLink(visitedLinkIndex);

         for(size_t contact = 0;
             contact < nrOfContactForLink;
             contact++ )
         {
             const UnknownWrenchContact & unknownWrench = unknownWrenches.contactWrench(visitedLinkIndex,contact);

             // We are estimate the force and the torque at the contact point,
             // so we need to compute the transform between a frame with the orientation
             // of the link frame and the origin given by the contact point
             const Transform & subModelBase_H_link = bufs.subModelBase_H_link(visitedLinkIndex);
             Transform link_H_contact = Transform(Rotation::Identity(),unknownWrench.contactPoint);
             Transform subModelBase_H_contact = subModelBase_H_link*link_H_contact;

             switch( unknownWrench.unknownType )
             {
                 case FULL_WRENCH:
                     Aeig.block<6,6>(0,AcolsToWrite) = toEigen(subModelBase_H_contact.asAdjointTransformWrench());
                     AcolsToWrite += 6;
                     break;
                 case PURE_FORCE:
                     Aeig.block<6,3>(0,AcolsToWrite) =
                         toEigen(subModelBase_H_contact.asAdjointTransformWrench()).block<6,3>(0,0);
                     AcolsToWrite += 3;
                     break;
                 case PURE_FORCE_WITH_KNOWN_DIRECTION:
                 {
                     Eigen::Matrix<double,6,1> fDir;
                     fDir.setZero();
                     fDir.segment<3>(0) = toEigen(unknownWrench.forceDirection);
                     Aeig.block<6,1>(0,AcolsToWrite) =
                         toEigen(subModelBase_H_contact.asAdjointTransformWrench())*fDir;
                     AcolsToWrite += 1;
                 }
                     break;
                 case NO_UNKNOWNS:
                 {
                     // AcolsToWrite += 1;
                     toEigen(bufs.b[subModelIndex]) += -toEigen(subModelBase_H_link*(unknownWrench.knownWrench));
                 }
                     break;
                 default:
                     assert(false);
                     break;
             }
         }
     }
}

void storeResultsOfEstimation(const Traversal& traversal,
                              const LinkUnknownWrenchContacts& unknownWrenches,
                              const size_t subModelIndex,
                              const estimateExternalWrenchesBuffers& bufs,
                                    LinkContactWrenches& outputContactWrenches)
{
    size_t nextUnknownToRead = 0;
    for(int traversalEl = traversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex    visitedLinkIndex = visitedLink->getIndex();

        size_t nrOfContactForLink = unknownWrenches.getNrOfContactsForLink(visitedLinkIndex);

        outputContactWrenches.setNrOfContactsForLink(visitedLinkIndex,nrOfContactForLink);

        for(size_t contact = 0;
            contact < nrOfContactForLink;
            contact++ )
        {
            const UnknownWrenchContact & unknownWrench = unknownWrenches.contactWrench(visitedLinkIndex,contact);
            Wrench & estimatedWrench = outputContactWrenches.contactWrench(visitedLinkIndex,contact).contactWrench();
            outputContactWrenches.contactWrench(visitedLinkIndex,contact).contactPoint() = unknownWrench.contactPoint;

            // Preserve the contact ID number
            outputContactWrenches.contactWrench(visitedLinkIndex,contact).contactId() = unknownWrench.contactId;

            switch( unknownWrench.unknownType )
            {
                case FULL_WRENCH:
                    estimatedWrench.getLinearVec3()(0) = bufs.x[subModelIndex](nextUnknownToRead+0);
                    estimatedWrench.getLinearVec3()(1) = bufs.x[subModelIndex](nextUnknownToRead+1);
                    estimatedWrench.getLinearVec3()(2) = bufs.x[subModelIndex](nextUnknownToRead+2);
                    estimatedWrench.getAngularVec3()(0) = bufs.x[subModelIndex](nextUnknownToRead+3);
                    estimatedWrench.getAngularVec3()(1) = bufs.x[subModelIndex](nextUnknownToRead+4);
                    estimatedWrench.getAngularVec3()(2) = bufs.x[subModelIndex](nextUnknownToRead+5);
                    nextUnknownToRead += 6;
                    break;
                case PURE_FORCE:
                    estimatedWrench.getLinearVec3()(0) = bufs.x[subModelIndex](nextUnknownToRead+0);
                    estimatedWrench.getLinearVec3()(1) = bufs.x[subModelIndex](nextUnknownToRead+1);
                    estimatedWrench.getLinearVec3()(2) = bufs.x[subModelIndex](nextUnknownToRead+2);
                    estimatedWrench.getAngularVec3().zero();
                    nextUnknownToRead += 3;
                    break;
                case PURE_FORCE_WITH_KNOWN_DIRECTION:
                    toEigen(estimatedWrench.getLinearVec3()) =
                        bufs.x[subModelIndex](nextUnknownToRead+0)*toEigen(unknownWrench.forceDirection);
                    estimatedWrench.getAngularVec3().zero();
                    nextUnknownToRead += 1;
                    break;
                case NO_UNKNOWNS:
                    estimatedWrench = unknownWrench.knownWrench;
                    break;
                default:
                    assert(false);
                    break;
            }
        }
    }
}

bool estimateExternalWrenchesWithoutInternalFT(const Model& model,
                                               const Traversal& traversal,
                                               const LinkUnknownWrenchContacts& unknownWrenches,
                                               const JointPosDoubleArray& jointPos,
                                               const LinkVelArray& linkVel,
                                               const LinkAccArray& linkProperAcc,
                                                     estimateExternalWrenchesBuffers& bufs,
                                                     LinkContactWrenches& outputContactWrenches)
{
    /**< value extracted from old iDynContact */
    double tol = 1e-7;

    // make sure that the bufs has the right size
    if( bufs.getNrOfLinks() != model.getNrOfLinks() ||
        bufs.getNrOfSubModels() != 1 )
    {
        reportError("","estimateExternalWrenchesWithoutInternalFT","input buffer has wrong size.");
        return false;
    }

    // First compute the known term of the estimation for each link:
   // this loop is similar to the dynamic phase of the RNEA
   // \todo pimp up performance as done in RNEADynamicPhase
   Wrench knownTerms = computeKnownTermsOfEstimationEquationWithoutInternalFT(model,traversal,
                                                                              jointPos,linkVel,linkProperAcc,bufs);

   // All the methods are designed to act on submodels, but in this case
   // we are considering the full model
   size_t subModelIndex = 0;

   // Copy knownTerms in the buffers used for estimation
   bufs.b[subModelIndex] = knownTerms.asVector();

   // Now we compute the A matrix
   computeMatrixOfEstimationEquationAndExtWrenchKnownTerms(model,traversal,unknownWrenches,jointPos,subModelIndex,bufs);

   // If A has no unkowns then pseudoInverse can not be computed
   // In that case, we do not compute the x vector because it will have zero elements 
   if (bufs.A[subModelIndex].rows() > 0 && bufs.A[subModelIndex].cols() > 0) {
       // Now we compute the pseudo inverse
       // pseudoInverse(toEigen(bufs.A[subModelIndex]),
       //               toEigen(bufs.pinvA[subModelIndex]),
       //               tol);

       // Now we compute the unknowns
       toEigen(bufs.x[subModelIndex]) = toEigen(bufs.A[subModelIndex]).colPivHouseholderQr().solve(toEigen(bufs.b[subModelIndex]));
   }

   // We copy the estimated unknowns in the outputContactWrenches
   // Note that the logic of conversion between input/output contacts should be
   // the same used before in computeMatrixOfEstimationEquation
   storeResultsOfEstimation(traversal,unknownWrenches,subModelIndex,bufs,outputContactWrenches);

   return true;
}


bool estimateExternalWrenches(const Model& model,
                              const SubModelDecomposition& subModels,
                              const SensorsList& sensors,
                              const LinkUnknownWrenchContacts& unknownWrenches,
                              const JointPosDoubleArray & jointPos,
                              const LinkVelArray& linkVel,
                              const LinkAccArray& linkProperAcc,
                              const SensorsMeasurements& ftSensorsMeasurements,
                                    estimateExternalWrenchesBuffers& bufs,
                                    LinkContactWrenches& outputContactWrenches)
{
    /**< value extracted from old iDynContact */
    double tol = 1e-7;

    // Resize the output data structure
    outputContactWrenches.resize(model);

    // Solve the problem for each submodel
    for(size_t sm=0; sm < subModels.getNrOfSubModels(); sm++ )
    {
        // Number of unknowns for this submodel
        const Traversal & subModelTraversal = subModels.getTraversal(sm);

        // First compute the known term of the estimation for each link:
        // this loop is similar to the dynamic phase of the RNEA
        // \todo pimp up performance as done in RNEADynamicPhase
        Wrench knownTerms = computeKnownTermsOfEstimationEquationWithInternalFT(model,subModelTraversal,
                                sensors,jointPos,linkVel,linkProperAcc,ftSensorsMeasurements,bufs);

        // Copy knownTerms in the buffers used for estimation
        bufs.b[sm] = knownTerms.asVector();

        // Now we compute the A matrix
        computeMatrixOfEstimationEquationAndExtWrenchKnownTerms(model,subModelTraversal,unknownWrenches,jointPos,sm,bufs);

        // If A has no unkowns then pseudoInverse can not be computed
        // In that case, we do not compute the x vector because it will have zero elements 
        if (bufs.A[sm].rows() > 0 && bufs.A[sm].cols() > 0) {
	    toEigen(bufs.x[sm]) = toEigen(bufs.A[sm]).colPivHouseholderQr().solve(toEigen(bufs.b[sm]));	    
        }

        // Check if there are any nan in the estimation results
        bool someResultIsNan = false;
        for(size_t i=0; i < bufs.x[sm].size(); i++)
        {
            if( std::isnan(bufs.x[sm](i)) )
            {
                someResultIsNan = true;
                break;
            }
        }

        if (someResultIsNan)
        {
            reportError("", "estimateExternalWrenches", "NaN found in estimation result, estimation failed");
            return false;
        }

        // We copy the estimated unknowns in the outputContactWrenches
        // Note that the logic of conversion between input/output contacts should be
        // the same used before in computeMatrixOfEstimationEquation
        storeResultsOfEstimation(subModelTraversal,unknownWrenches,sm,bufs,outputContactWrenches);
    }

    return true;
}

bool dynamicsEstimationForwardVelAccKinematics(const iDynTree::Model & /*model*/,
                                               const iDynTree::Traversal & traversal,
                                               const Vector3 & base_classicalProperAcc,
                                               const Vector3 & base_angularVel,
                                               const Vector3 & base_angularAcc,
                                               const iDynTree::JointPosDoubleArray & jointPos,
                                               const iDynTree::JointDOFsDoubleArray & jointVel,
                                               const iDynTree::JointDOFsDoubleArray & jointAcc,
                                                     iDynTree::LinkVelArray & linkVel,
                                                     iDynTree::LinkAccArray  & linkProperAcc)
{
    bool retValue = true;

    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        if( parentLink == 0 )
        {
            // If the visited link is the base, we can set the base velocity and proper acceleration
            // from the input base information

            // the dynamics is invariant to a linear velocity offset, so we can put an arbitrary
            // value for the linear part of the twist: we choose to set it to zero for convenience,
            // but please note that this **does not** mean that we are assuming that the body
            // has a zero velocity with respect to a earth-fixed frame
            LinearMotionVector3 linVel;
            linVel.zero();
            linkVel(visitedLink->getIndex()).setLinearVec3(linVel);

            // We have the input angular velocity
            AngularMotionVector3 angVel(base_angularVel.data(),3);
            linkVel(visitedLink->getIndex()).setAngularVec3(angVel);

            // We don't need to convert the proper classical acceleration
            // in spatial classical acceleration because the difference between
            // the two depends linearly on the linear part of the link velocity,
            // that we choose to be zero
            LinearMotionVector3 linProAcc(base_classicalProperAcc.data(),3);
            linkProperAcc(visitedLink->getIndex()).setLinearVec3(linProAcc);
            AngularMotionVector3 angAcc(base_angularAcc.data(),3);
            linkProperAcc(visitedLink->getIndex()).setAngularVec3(angAcc);
        }
        else
        {
            // Otherwise we compute the child velocity and acceleration from parent
            toParentJoint->computeChildVelAcc(jointPos,
                                              jointVel,
                                              jointAcc,
                                              linkVel,
                                              linkProperAcc,
                                              visitedLink->getIndex(),parentLink->getIndex());
        }

    }

    return retValue;

}

bool computeLinkNetWrenchesWithoutGravity(const Model& model,
                                          const LinkVelArray& linkVel,
                                          const LinkAccArray& linkProperAcc,
                                                LinkNetTotalWrenchesWithoutGravity& linkNetWrenchesWithoutGravity)
{
     // Note that we are not using a Traversal here: the main reason
     // is that given that the computation that we are doing (once we have the linkVel and linkProperAcc
     // does not depend on the topology, so we can visit the links in any possible order
     for(LinkIndex visitedLinkIndex = 0; visitedLinkIndex < static_cast<LinkIndex>(model.getNrOfLinks()); visitedLinkIndex++)
     {
         LinkConstPtr visitedLink = model.getLink(visitedLinkIndex);

         const iDynTree::SpatialInertia & I = visitedLink->getInertia();
         const iDynTree::SpatialAcc     & properAcc = linkProperAcc(visitedLinkIndex);
         const iDynTree::Twist          & v = linkVel(visitedLinkIndex);
         linkNetWrenchesWithoutGravity(visitedLinkIndex) = I*properAcc + v*(I*v);
     }

     return true;
}



bool dynamicsEstimationForwardVelKinematics(const Model & /*model*/,
                                            const Traversal & traversal,
                                            const Vector3 & base_angularVel,
                                            const JointPosDoubleArray & jointPos,
                                            const JointDOFsDoubleArray & jointVel,
                                                  LinkVelArray & linkVel)
{
    bool retValue = true;

    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        if( parentLink == 0 )
        {
            // If the visited link is the base, we can set the base velocity and proper acceleration
            // from the input base information

            // the dynamics is invariant to a linear velocity offset, so we can put an arbitrary
            // value for the linear part of the twist: we choose to set it to zero for convenience,
            // but please note that this **does not** mean that we are assuming that the body
            // has a zero velocity with respect to a earth-fixed frame
            LinearMotionVector3 linVel;
            linVel.zero();
            linkVel(visitedLink->getIndex()).setLinearVec3(linVel);

            // We have the input angular velocity
            AngularMotionVector3 angVel(base_angularVel.data(),3);
            linkVel(visitedLink->getIndex()).setAngularVec3(angVel);
        }
        else
        {
            // Otherwise we compute the child velocity and acceleration from parent
            toParentJoint->computeChildVel(jointPos,
                                           jointVel,
                                            linkVel,
                                            visitedLink->getIndex(),parentLink->getIndex());
        }

    }

    return retValue;

}

bool estimateLinkContactWrenchesFromLinkNetExternalWrenches(const Model& model,
                                                            const LinkUnknownWrenchContacts& unknownWrenches,
                                                            const LinkNetExternalWrenches& netExtWrenches,
                                                                  LinkContactWrenches & outputContactWrenches)
{
    for (LinkIndex lnkIdx=0; lnkIdx < static_cast<LinkIndex>(model.getNrOfLinks()); lnkIdx++)
    {
        if (unknownWrenches.getNrOfContactsForLink(lnkIdx) > 1)
        {
            std::stringstream ss;
            ss << "estimateLinkContactWrenchesFromLinkNetExternalWrenches is currently in testing phase, and does not support "
               << " handling multiple contacts in one link. Skipping contacts on link " << model.getLinkName(lnkIdx);
            reportWarning("","estimateLinkContactWrenchesFromLinkNetExternalWrenches", ss.str().c_str());
            outputContactWrenches.setNrOfContactsForLink(lnkIdx, 0);
            continue;
        }

        if (unknownWrenches.getNrOfContactsForLink(lnkIdx) == 1 &&
            unknownWrenches.contactWrench(lnkIdx, 0).unknownType != FULL_WRENCH)
        {
            std::stringstream ss;
            ss << "estimateLinkContactWrenchesFromLinkNetExternalWrenches is currently in testing phase, and does not support "
               << " contact of type different from FULL_WRENCH, skipping contacts  " << model.getLinkName(lnkIdx);
            reportWarning("","estimateLinkContactWrenchesFromLinkNetExternalWrenches", ss.str().c_str());
            outputContactWrenches.setNrOfContactsForLink(lnkIdx, 0);
            continue;
        }

        // If the link has no contact, we can just go to next link
        if (unknownWrenches.getNrOfContactsForLink(lnkIdx) == 0)
        {
            continue;
        }

        const UnknownWrenchContact& unknownWrench = unknownWrenches.contactWrench(lnkIdx, 0);
        outputContactWrenches.setNrOfContactsForLink(lnkIdx, unknownWrenches.getNrOfContactsForLink(lnkIdx));
        ContactWrench& knownWrench          = outputContactWrenches.contactWrench(lnkIdx, 0);

        Transform link_H_contact = Transform(Rotation::Identity(), unknownWrench.contactPoint);

        // Transform the wrench from the link frame to the contact frame
        knownWrench.contactPoint() = unknownWrench.contactPoint;
        knownWrench.contactId()    = unknownWrench.contactId;
        knownWrench.contactWrench() = link_H_contact.inverse()*netExtWrenches(lnkIdx);
    }

    return true;
}


}

