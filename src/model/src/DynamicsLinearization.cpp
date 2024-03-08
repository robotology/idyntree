// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause



#include <iDynTree/DynamicsLinearization.h>

#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>

#include <iDynTree/FreeFloatingState.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/TransformDerivative.h>

#include <Eigen/Core>

namespace iDynTree
{

ForwardDynamicsLinearizationInternalBuffers::ForwardDynamicsLinearizationInternalBuffers(const Model& model)
{
    resize(model);
}

void ForwardDynamicsLinearizationInternalBuffers::resize(const Model& model)
{
    aba.resize(model);

    dPos.resize(model.getNrOfDOFs());
    dVel.resize(model.getNrOfDOFs());

    for(size_t dof = 0; dof < model.getNrOfDOFs(); dof++)
    {
        dPos[dof].resize(model);
        dVel[dof].resize(model);
    }

    linkPos.resize(model.getNrOfLinks());
    dVb_linkBiasWrench.resize(model.getNrOfLinks());
    dVb_u.resize(model.getNrOfLinks());
    dVb_linkBiasAcceleration.resize(model.getNrOfLinks());
    dVb_linksAccelerations.resize(model.getNrOfLinks());
    dVl_linkLocalBiasWrench.resize(model.getNrOfLinks());

    for(size_t link = 0; link < model.getNrOfLinks(); link++ )
    {
        linkPos(link) = iDynTree::Transform::Identity();
        dVb_linkBiasWrench[link].zero();
        dVb_u[link].zero();
        dVb_linkBiasAcceleration[link].zero();
        dVb_linksAccelerations[link].zero();
        dVl_linkLocalBiasWrench[link].zero();
    }

}

FreeFloatingStateLinearization::FreeFloatingStateLinearization(const Model& model)
{
    this->resize(model);
}

void FreeFloatingStateLinearization::resize(const Model& model)
{
    MatrixDynSize::resize(2*(6+model.getNrOfDOFs()),2*(6+model.getNrOfDOFs()));
    this->zero();
}

void ForwardDynamicsLinearizationWrtJointPos(const Model& model,
                                             const Traversal& traversal,
                                             const FreeFloatingPos& robotPos,
                                             const FreeFloatingVel& robotVel,
                                             const LinkNetExternalWrenches& linkExtWrenches,
                                             const JointDOFsDoubleArray& jointTorques,
                                             const size_t dofDeriv,
                                             ForwardDynamicsLinearizationInternalBuffers & bufs,
                                             FreeFloatingAcc& robotAcc,
                                             FreeFloatingStateLinearization& A)
{
    /**
     * For linearization with respect to the degrees of freedom dofDeriv,
     * the only transform between two neighbor links that depends of the
     * value of the degree of freedom dofDeriv is the one between the two links
     * connected by the joint of the dof dofDeriv.
     * Given that it is used extensivly in the loop, we store it.
     */
    TransformDerivative visited_dX_parent;

    /**
     * Forward pass: compute the derivative of link velocities and the link bias accelerations
     * and initialize the derivatives of Articulated Body Inertia and the articulated bias wrench.
     */
    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        // Propagate velocities and bias accelerations

        // If the visitedLink is the base one, initialize the derivative wrt to joint position with zero vector
        if( parentLink == 0 )
        {
            assert(visitedLinkIndex >= 0 && visitedLinkIndex < (LinkIndex)model.getNrOfLinks());
            bufs.dPos[dofDeriv].linksVel(visitedLinkIndex) = Twist::Zero();
            bufs.dPos[dofDeriv].linksBiasAcceleration(visitedLinkIndex) = SpatialAcc::Zero();
        }
        else
        {
            LinkIndex    parentLinkIndex = parentLink->getIndex();
            // If it is a fixed joint we propagate the derivative
            if( toParentJoint->getNrOfDOFs() == 0 )
            {
                bufs.dPos[dofDeriv].linksVel(visitedLinkIndex) =
                    toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLink->getIndex())*bufs.dPos[dofDeriv].linksVel(parentLinkIndex);
                bufs.dPos[dofDeriv].linksBiasAcceleration(visitedLinkIndex) = SpatialAcc::Zero();
            }
            else
            {
                size_t dofIndex = toParentJoint->getDOFsOffset();

                Transform visited_X_parent = toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex);

                if( dofDeriv == dofIndex )
                {
                    // if the considered link is connected to its parent with the joint wrt
                    // to which we are doing the derivation, we need to handle the special case
                    visited_dX_parent =
                        toParentJoint->getTransformDerivative(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex,0);

                    Eigen::Matrix<double,6,1> tmp
                        = toEigen(visited_dX_parent.asAdjointTransformDerivative(visited_X_parent))*
                          toEigen(bufs.aba.linksVel(visitedLinkIndex));

                    toEigen(bufs.dPos[dofDeriv].linksVel(visitedLinkIndex).getLinearVec3()) = tmp.segment<3>(0);
                    toEigen(bufs.dPos[dofDeriv].linksVel(visitedLinkIndex).getAngularVec3()) = tmp.segment<3>(3);
                }
                else
                {
                    bufs.dPos[dofDeriv].linksVel(visitedLinkIndex) =
                        visited_X_parent*bufs.dPos[dofDeriv].linksVel(parentLinkIndex);
                }

                Twist vj = bufs.aba.S(dofIndex)*robotVel.jointVel()(dofIndex);
                bufs.dPos[dofDeriv].linksBiasAcceleration(visitedLinkIndex) = bufs.dPos[dofDeriv].linksVel(visitedLinkIndex)*vj;
            }
        }

        // Initialize Articulated Body Inertia
        bufs.dPos[dofDeriv].linkABIs(visitedLinkIndex).zero();

        // We fill the buffer of the derivative of biasWrench with respect to link velocity
        // that will be used by everyone
        const iDynTree::SpatialInertia & M = visitedLink->getInertia();
        toEigen(bufs.dVl_linkLocalBiasWrench[visitedLinkIndex]) = toEigen(M.biasWrenchDerivative(bufs.aba.linksVel(visitedLinkIndex)));

        // Initialize bias force (note that we assume that the external frames are
        // expressed in a local frame, differently from Featherstone 2008
        Eigen::Matrix<double,6,1> tmp = toEigen(bufs.dVl_linkLocalBiasWrench[visitedLinkIndex])*toEigen(bufs.dPos[dofDeriv].linksVel(visitedLinkIndex));

        toEigen(bufs.dPos[dofDeriv].linksBiasWrench(visitedLinkIndex).getLinearVec3()) = tmp.segment<3>(0);
        toEigen(bufs.dPos[dofDeriv].linksBiasWrench(visitedLinkIndex).getAngularVec3()) = tmp.segment<3>(3);
    }

    /*
     * Backward pass: recursivly compute
     * the derivative wrt joint position of
     * the articulated body inertia and the articulated body bias wrench.
     */
    for(int traversalEl = traversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex    visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        if( parentLink )
        {
            ArticulatedBodyInertia dPos_Ia;
            ArticulatedBodyInertia Ia;
            Wrench dPos_pa;

            // For now we support only 0 and 1 dof joints

            // for 1 dof joints, the articulated inertia
            // need to be propagated to the parent considering
            // the joints
            if( toParentJoint->getNrOfDOFs() > 0 )
            {
                assert(toParentJoint->getNrOfDOFs()==1);
                size_t dofIndex = toParentJoint->getDOFsOffset();
                bufs.dPos[dofDeriv].U(dofIndex) = bufs.dPos[dofDeriv].linkABIs(visitedLinkIndex)*bufs.aba.S(dofIndex);
                bufs.dPos[dofDeriv].D(dofIndex) = bufs.aba.S(dofIndex).dot(bufs.dPos[dofDeriv].U(dofIndex));
                bufs.dPos[dofDeriv].u(dofIndex) =  - bufs.aba.S(dofIndex).dot(bufs.dPos[dofDeriv].linksBiasWrench(visitedLinkIndex));

                double invD = 1/(bufs.aba.D(dofIndex));
                double d_invD = - invD * bufs.dPos[dofDeriv].D(dofIndex) * invD;

                dPos_Ia = bufs.dPos[dofDeriv].linkABIs(visitedLinkIndex) -
                        ArticulatedBodyInertia::ABADyadHelperLin(bufs.aba.U(dofIndex),invD,
                                                                 bufs.dPos[dofDeriv].U(dofIndex),d_invD);
                Ia = bufs.aba.linkABIs(visitedLinkIndex)
                    - ArticulatedBodyInertia::ABADyadHelper(bufs.aba.U(dofIndex),bufs.aba.D(dofIndex));
                dPos_pa    =  bufs.dPos[dofDeriv].linksBiasWrench(visitedLinkIndex)
                                     + dPos_Ia*bufs.aba.linksBiasAcceleration(visitedLinkIndex)
                                     + Ia*bufs.dPos[dofDeriv].linksBiasAcceleration(visitedLinkIndex)
                                     + bufs.dPos[dofDeriv].U(dofIndex)*(bufs.aba.u(dofIndex)*invD)
                                     + bufs.aba.U(dofIndex)*(bufs.dPos[dofDeriv].u(dofIndex)*invD + bufs.aba.u(dofIndex)*d_invD);

            }

            // For fixed joints, we just need to propagate
            // the articulated quantities without considering the
            // joint
            if( toParentJoint->getNrOfDOFs() == 0 )
            {
                dPos_Ia = bufs.dPos[dofDeriv].linkABIs(visitedLinkIndex);
                Ia = bufs.aba.linkABIs(visitedLinkIndex);
                dPos_pa   =   bufs.dPos[dofDeriv].linksBiasWrench(visitedLinkIndex)
                               + (Ia*bufs.dPos[dofDeriv].linksBiasAcceleration(visitedLinkIndex))
                               + (dPos_Ia*bufs.aba.linksBiasAcceleration(visitedLinkIndex));
            }

            //bufs.pa(visitedLinkIndex) = pa;

            // Propagate
            LinkIndex parentLinkIndex = parentLink->getIndex();
            Transform parent_X_visited = toParentJoint->getTransform(robotPos.jointPos(),parentLinkIndex,visitedLinkIndex);
            bufs.dPos[dofDeriv].linkABIs(parentLinkIndex)        += parent_X_visited*dPos_Ia;
            bufs.dPos[dofDeriv].linksBiasWrench(parentLinkIndex) = bufs.dPos[dofDeriv].linksBiasWrench(parentLinkIndex) + parent_X_visited*dPos_pa;

            // if the visited link is connected to the parent with the joint wrt we are computing the
            // derivative we have to add some additional terms relative to the derivative of the transforms
            size_t dofIndex = toParentJoint->getDOFsOffset();
            if( dofIndex == dofDeriv )
            {
                const Transform & visited_X_parent = toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex);
                TransformDerivative  parent_dX_visited = visited_dX_parent.derivativeOfInverse(visited_X_parent);

                bufs.dPos[dofDeriv].linkABIs(parentLinkIndex) += parent_dX_visited.transform(parent_X_visited,Ia);

                SpatialForceVector pa =   bufs.aba.linksBiasWrench(visitedLinkIndex)
                                     + Ia*bufs.aba.linksBiasAcceleration(visitedLinkIndex)
                                     + bufs.aba.U(dofIndex)*(bufs.aba.u(dofIndex)/bufs.aba.D(dofIndex));

                bufs.dPos[dofDeriv].linksBiasWrench(parentLinkIndex) = bufs.dPos[dofDeriv].linksBiasWrench(parentLinkIndex)
                                                                      + parent_dX_visited.transform(parent_X_visited,pa);
            }

        }
    }

    /**
     * Second forward pass: find robot accelerations
     *
     */
    // \todo TODO Handle gravity
    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
       LinkConstPtr visitedLink = traversal.getLink(traversalEl);
       LinkIndex visitedLinkIndex = visitedLink->getIndex();
       LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
       IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

       if( parentLink == 0 )
       {
           // Preliminary step: find base acceleration
           Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");;
           Matrix6x6 inverseInertia = bufs.aba.linkABIs(visitedLinkIndex).getInverse();
           std::cout << "Inertia: " << toEigen(bufs.aba.linkABIs(visitedLinkIndex).asMatrix()).format(OctaveFmt) << std::endl;
           std::cout << "Inverse Inertia: " << inverseInertia.toString() << std::endl;
           Matrix6x6 dPos_inertia = bufs.dPos[dofDeriv].linkABIs(visitedLinkIndex).asMatrix();
           Matrix6x6 dPos_inverseInertia;
           toEigen(dPos_inverseInertia) = -toEigen(inverseInertia)*toEigen(dPos_inertia)*toEigen(dPos_inverseInertia);

           Eigen::Matrix<double,6,1> tmp = -(toEigen(dPos_inverseInertia)*toEigen(bufs.aba.linksBiasWrench(visitedLinkIndex))
                                                                        +toEigen(inverseInertia)*toEigen(bufs.dPos[dofDeriv].linksBiasWrench(visitedLinkIndex)));

           toEigen(bufs.dPos[dofDeriv].linksAccelerations(visitedLinkIndex).getLinearVec3()) = tmp.segment<3>(0);
           toEigen(bufs.dPos[dofDeriv].linksAccelerations(visitedLinkIndex).getAngularVec3()) = tmp.segment<3>(3);

           toEigen(A).block<6,1>(6+model.getNrOfDOFs(),6+dofDeriv) = toEigen(bufs.dPos[dofDeriv].linksAccelerations(visitedLinkIndex));
       }
       else
       {
           LinkIndex    parentLinkIndex = parentLink->getIndex();
           if( toParentJoint->getNrOfDOFs() > 0 )
           {
               const Transform & visited_X_parent = toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex);
               size_t dofIndex = toParentJoint->getDOFsOffset();
               assert(toParentJoint->getNrOfDOFs()==1);
               bufs.dPos[dofDeriv].linksAccelerations(visitedLinkIndex) =
                   visited_X_parent*bufs.dPos[dofDeriv].linksAccelerations(parentLinkIndex)
                   + bufs.dPos[dofDeriv].linksBiasAcceleration(visitedLinkIndex);

               if( dofIndex == dofDeriv )
               {
                   // account for derivative with respect to position
                   bufs.dPos[dofDeriv].linksAccelerations(visitedLinkIndex) =
                    bufs.dPos[dofDeriv].linksAccelerations(visitedLinkIndex) +
                    visited_dX_parent.transform(visited_X_parent,bufs.aba.linksBiasAcceleration(visitedLinkIndex));
               }

               double invD = 1/(bufs.aba.D(dofIndex));
               double d_invD = - invD * bufs.dPos[dofDeriv].D(dofIndex) * invD;
               double dPos_ddq =
                    (bufs.aba.u(dofIndex)-bufs.aba.U(dofIndex).dot(bufs.aba.linksAccelerations(visitedLinkIndex)))*d_invD+
                    (bufs.dPos[dofDeriv].u(dofIndex)
                        -bufs.dPos[dofDeriv].U(dofIndex).dot(bufs.aba.linksAccelerations(visitedLinkIndex))
                        -bufs.aba.U(dofIndex).dot(bufs.dPos[dofDeriv].linksAccelerations(visitedLinkIndex)))*invD;

               A(6+model.getNrOfDOFs()+6+dofIndex,6+dofDeriv) = dPos_ddq;

               bufs.dPos[dofDeriv].linksAccelerations(visitedLinkIndex) =
                bufs.dPos[dofDeriv].linksAccelerations(visitedLinkIndex) +
                bufs.aba.S(dofIndex)*dPos_ddq;
           }
           else
           {
               //for fixed joints we just propagate the acceleration
               const Transform & visited_X_parent = toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex);
               bufs.dPos[dofDeriv].linksAccelerations(visitedLinkIndex) =
                   visited_X_parent*bufs.dPos[dofDeriv].linksAccelerations(parentLinkIndex)
                   + bufs.dPos[dofDeriv].linksBiasAcceleration(visitedLinkIndex);
           }
       }
    }

}

void ForwardDynamicsLinearizationWrtBaseTwist(const Model& model,
                                              const Traversal& traversal,
                                              const FreeFloatingPos& robotPos,
                                              const FreeFloatingVel& robotVel,
                                              const LinkNetExternalWrenches& linkExtWrenches,
                                              const JointDOFsDoubleArray& jointTorques,
                                              ForwardDynamicsLinearizationInternalBuffers & bufs,
                                              FreeFloatingAcc& robotAcc,
                                              FreeFloatingStateLinearization& A)
{
    //std::cout << "ForwardDynamicsLinearizationWrtBaseTwist A : " << A.toString() << std::endl;
    /**
     * Forward pass: compute the derivatives of  link velocities and of link bias accelerations
     * with respect to the base twist.
     * In particur, the derivative of the linkVelocity[l] with respect to the base velocity
     * is just the \f$ ~^b X_l \f$ transform, so we just compute that transform for the derivative
     * of link velocity with respect to base velocity.
     * We don't such a strong structure for the derivative with respect to the link bias acceleration,
     * so store it as a 6x6 matrix.
     */
    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        // Propagate the derivatives of velocities and bias accelerations

        // If the visitedLink is the base one, initialize the derivative of the link velocity
        // with respect to the base velocity to identiy
        if( parentLink == 0 )
        {
            bufs.linkPos(visitedLinkIndex) = iDynTree::Transform::Identity();
            bufs.dVb_linkBiasAcceleration[visitedLinkIndex].zero();
        }
        else
        {
            LinkIndex    parentLinkIndex = parentLink->getIndex();
            bufs.linkPos(visitedLinkIndex) =
                    toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLink->getIndex())*bufs.linkPos(parentLinkIndex);

            if( toParentJoint->getNrOfDOFs() == 0 )
            {
                bufs.dVb_linkBiasAcceleration[visitedLinkIndex].zero();
            }
            else
            {
               size_t dofIndex = toParentJoint->getDOFsOffset();
               double dql      = robotVel.jointVel()(dofIndex);
               iDynTree::SpatialMotionVector & S = bufs.aba.S(dofIndex);

               toEigen(bufs.dVb_linkBiasAcceleration[visitedLinkIndex]) = toEigen((S*(-dql)).asCrossProductMatrix())*toEigen( bufs.linkPos(visitedLinkIndex).asAdjointTransform());
            }
        }

       //std::cout << " dVl_linkLocalBiasWrench : " << bufs.dVl_linkLocalBiasWrench[visitedLinkIndex].toString() << std::endl;
       toEigen(bufs.dVb_linkBiasWrench[visitedLinkIndex]) = toEigen(bufs.dVl_linkLocalBiasWrench[visitedLinkIndex])*toEigen( bufs.linkPos(visitedLinkIndex).asAdjointTransform());


    }

    /*
     * Backward pass: recursivly compute
     * the derivative of articulated body bias wrench
     * with respect to the base velocity.
     *
     */
    for(int traversalEl = traversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex    visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);


        if( parentLink )
        {
            // For now we support only 0 and 1 dof joints
            Matrix6x6 dVb_pa;
            ArticulatedBodyInertia Ia;

            // for 1 dof joints, the articulated inertia
            // need to be propagated to the parent considering
            // the joints
            if( toParentJoint->getNrOfDOFs() > 0 )
            {
                assert(toParentJoint->getNrOfDOFs()==1);
                size_t dofIndex = toParentJoint->getDOFsOffset();

                Ia = bufs.aba.linkABIs(visitedLinkIndex)
                    - ArticulatedBodyInertia::ABADyadHelper(bufs.aba.U(dofIndex),bufs.aba.D(dofIndex));

                toEigen(bufs.dVb_u[dofIndex]) = -toEigen(bufs.aba.S(dofIndex)).transpose()*toEigen(bufs.dVb_linkBiasWrench[visitedLinkIndex]);
                toEigen(dVb_pa)  =  toEigen(bufs.dVb_linkBiasWrench[visitedLinkIndex])
                                     + toEigen(Ia.asMatrix())*toEigen(bufs.dVb_linkBiasAcceleration[visitedLinkIndex])
                                     + toEigen(bufs.aba.U(dofIndex))*toEigen(bufs.dVb_u[dofIndex])/bufs.aba.D(dofIndex);

            }

            // For fixed joints, we just need to propate
            // the articulated quantities without considering the
            // joint
            if( toParentJoint->getNrOfDOFs() == 0 )
            {
                Ia = bufs.aba.linkABIs(visitedLinkIndex);
                toEigen(dVb_pa)   =  toEigen(bufs.dVb_linkBiasWrench[visitedLinkIndex])
                                     + toEigen(Ia.asMatrix())*toEigen(bufs.dVb_linkBiasAcceleration[visitedLinkIndex]);
            }

            // Propagate
            LinkIndex parentLinkIndex = parentLink->getIndex();
            const Transform & parent_X_visited = toParentJoint->getTransform(robotPos.jointPos(),parentLinkIndex,visitedLinkIndex);
            toEigen(bufs.dVb_linkBiasWrench[parentLinkIndex]) += toEigen(parent_X_visited.asAdjointTransformWrench())*toEigen(dVb_pa);
        }
    }


    /**
     * Second forward pass: find robot accelerations
     *
     */
    // \todo TODO Handle gravity
    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
       LinkConstPtr visitedLink = traversal.getLink(traversalEl);
       LinkIndex visitedLinkIndex = visitedLink->getIndex();
       LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
       IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

       if( parentLink == 0 )
       {
           // Preliminary step: find base acceleration
           //std::cout << "bufs.dVb_linkBiasWrench : "<<  bufs.dVb_linkBiasWrench[visitedLinkIndex].toString() << std::endl;
           toEigen(bufs.dVb_linksAccelerations[visitedLinkIndex]) = -toEigen(bufs.aba.linkABIs(visitedLinkIndex).asMatrix()).householderQr().solve(toEigen(bufs.dVb_linkBiasWrench[visitedLinkIndex]));
           // Start storing the result in the linearization matrix
           toEigen(A).block<6,6>(6+model.getNrOfDOFs(),6+model.getNrOfDOFs()) = toEigen(bufs.dVb_linksAccelerations[visitedLinkIndex]);
           /*
           std::cout << "Acceleration setted : " << std::endl;
           std::cout << toEigen(bufs.dVb_linksAccelerations[visitedLinkIndex]) << std::endl;
           std::cout << "Resulting A " << std::endl;
           */
        }
       else
       {
           LinkIndex    parentLinkIndex = parentLink->getIndex();
           if( toParentJoint->getNrOfDOFs() > 0 )
           {
               size_t dofIndex = toParentJoint->getDOFsOffset();
               assert(toParentJoint->getNrOfDOFs()==1);
               toEigen(bufs.dVb_linksAccelerations[visitedLinkIndex]) =
                   toEigen(toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex).asAdjointTransform())*toEigen(bufs.dVb_linksAccelerations[parentLinkIndex])
                   + toEigen(bufs.dVb_linkBiasAcceleration[visitedLinkIndex]);
               toEigen(A).block<1,6>(6+model.getNrOfDOFs()+6+dofIndex,6+model.getNrOfDOFs())  = (toEigen(bufs.dVb_u[dofIndex]) - toEigen(bufs.aba.U(dofIndex)).transpose()*toEigen(bufs.dVb_linksAccelerations[visitedLinkIndex]))/bufs.aba.D(dofIndex);
               toEigen(bufs.dVb_linksAccelerations[visitedLinkIndex]) = toEigen(bufs.dVb_linksAccelerations[visitedLinkIndex]) + toEigen(bufs.aba.S(dofIndex))*toEigen(A).block<1,6>(6+model.getNrOfDOFs()+6+dofIndex,6+model.getNrOfDOFs());
           }
           else
           {
               //for fixed joints we just propagate the acceleration
               toEigen(bufs.dVb_linksAccelerations[visitedLinkIndex]) =
                   toEigen(toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex).asAdjointTransform())*toEigen(bufs.dVb_linksAccelerations[parentLinkIndex])
                   + toEigen(bufs.dVb_linkBiasAcceleration[visitedLinkIndex]);
           }
       }
    }
}

void ForwardDynamicsLinearizationWrtJointVel(const Model& model,
                                             const Traversal& traversal,
                                             const FreeFloatingPos& robotPos,
                                             const FreeFloatingVel& robotVel,
                                             const LinkNetExternalWrenches& linkExtWrenches,
                                             const JointDOFsDoubleArray& jointTorques,
                                             const size_t dofDeriv,
                                             ForwardDynamicsLinearizationInternalBuffers & bufs,
                                             FreeFloatingAcc& robotAcc,
                                             FreeFloatingStateLinearization& A)
{
    /**
     * Forward pass: compute the link velocities and the link bias accelerations
     * and initialize the Articulated Body Inertia and the articulated bias wrench.
     */
    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        // Propagate velocities and bias accelerations

        // If the visitedLink is the base one
        // initialize the derivative wrt to the joint velocity to zero
        if( parentLink == 0 )
        {
            assert(visitedLinkIndex >= 0 && visitedLinkIndex < (LinkIndex)model.getNrOfLinks());
            bufs.dVel[dofDeriv].linksVel(visitedLinkIndex) = Twist::Zero();
            bufs.dVel[dofDeriv].linksBiasAcceleration(visitedLinkIndex) = SpatialAcc::Zero();
        }
        else
        {
            LinkIndex    parentLinkIndex = parentLink->getIndex();
            // Otherwise we propagate velocity in the usual way
            if( toParentJoint->getNrOfDOFs() == 0 )
            {
                bufs.dVel[dofDeriv].linksVel(visitedLinkIndex) =
                    toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLink->getIndex())*bufs.dVel[dofDeriv].linksVel(parentLinkIndex);
                bufs.dVel[dofDeriv].linksBiasAcceleration(visitedLinkIndex) = SpatialAcc::Zero();
            }
            else
            {
                // The formula changes if the parent dof of this link is the
                // link wrt we are doing the derivative
                size_t dofIndex = toParentJoint->getDOFsOffset();

                if( dofDeriv == dofIndex )
                {
                    bufs.dVel[dofDeriv].linksVel(visitedLinkIndex) = bufs.aba.S(dofIndex);
                    bufs.dVel[dofDeriv].linksBiasAcceleration(visitedLinkIndex) = bufs.aba.linksVel(visitedLinkIndex)*bufs.aba.S(dofIndex);
                }
                else
                {
                    Twist vj = bufs.aba.S(dofIndex)*robotVel.jointVel()(dofIndex);
                    bufs.dVel[dofDeriv].linksVel(visitedLinkIndex) =
                        toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex)*bufs.dVel[dofDeriv].linksVel(parentLinkIndex);
                    bufs.dVel[dofDeriv].linksBiasAcceleration(visitedLinkIndex) = bufs.dVel[dofDeriv].linksVel(visitedLinkIndex)*vj;
                }
            }
        }

        // The dVl_linkLocalBiasWrench buffer should have been filled by the wrt joint pos
        // loop
        //const iDynTree::SpatialInertia & M = visitedLink->getInertia();
        //toEigen(bufs.dVl_linkLocalBiasWrench[visitedLinkIndex]) = toEigen(M.biasWrenchDerivative(bufs.aba.linksVel(visitedLinkIndex)));
        Eigen::Matrix<double,6,1> tmp = toEigen(bufs.dVl_linkLocalBiasWrench[visitedLinkIndex])*toEigen(bufs.dVel[dofDeriv].linksVel(visitedLinkIndex));

        toEigen(bufs.dVel[dofDeriv].linksBiasWrench(visitedLinkIndex).getLinearVec3()) = tmp.segment<3>(0);
        toEigen(bufs.dVel[dofDeriv].linksBiasWrench(visitedLinkIndex).getAngularVec3()) = tmp.segment<3>(3);
    }

    /*
     * Backward pass: recursivly compute
     * the articulated body inertia and the articulated body bias wrench.
     *
     */
    for(int traversalEl = traversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex    visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        if( parentLink )
        {
            ArticulatedBodyInertia Ia;
            Wrench dVel_dofDeriv_pa;

            // For now we support only 0 and 1 dof joints

            // for 1 dof joints, the articulated inertia
            // need to be propagated to the parent considering
            // the joints
            if( toParentJoint->getNrOfDOFs() > 0 )
            {
                assert(toParentJoint->getNrOfDOFs()==1);
                size_t dofIndex = toParentJoint->getDOFsOffset();
                bufs.dVel[dofDeriv].u(dofIndex) = -bufs.aba.S(dofIndex).dot(bufs.dVel[dofDeriv].linksBiasWrench(visitedLinkIndex));

                Ia = bufs.aba.linkABIs(visitedLinkIndex) - ArticulatedBodyInertia::ABADyadHelper(bufs.aba.U(dofIndex),bufs.aba.D(dofIndex));

                dVel_dofDeriv_pa   =   bufs.dVel[dofDeriv].linksBiasWrench(visitedLinkIndex)
                                     + Ia*bufs.dVel[dofDeriv].linksBiasAcceleration(visitedLinkIndex)
                                     + bufs.aba.U(dofIndex)*(bufs.dVel[dofDeriv].u(dofIndex)/bufs.aba.D(dofIndex));

            }

            // For fixed joints, we just need to propate
            // the articulated quantities without considering the
            // joint
            if( toParentJoint->getNrOfDOFs() == 0 )
            {
                Ia = bufs.aba.linkABIs(visitedLinkIndex);
                dVel_dofDeriv_pa   =   bufs.dVel[dofDeriv].linksBiasWrench(visitedLinkIndex)
                                     + Ia*bufs.dVel[dofDeriv].linksBiasAcceleration(visitedLinkIndex);
            }


            //bufs.dVel[dofDeriv].pa(visitedLinkIndex) = dVel_dofDeriv_pa;

            // Propagate
            LinkIndex parentLinkIndex = parentLink->getIndex();
            Transform parent_X_visited = toParentJoint->getTransform(robotPos.jointPos(),parentLinkIndex,visitedLinkIndex);
            bufs.dVel[dofDeriv].linksBiasWrench(parentLinkIndex) = bufs.dVel[dofDeriv].linksBiasWrench(parentLinkIndex) + parent_X_visited*dVel_dofDeriv_pa;
        }
    }

    /**
     * Second forward pass: find robot accelerations
     *
     */
    // \todo TODO Handle gravity
    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
       LinkConstPtr visitedLink = traversal.getLink(traversalEl);
       LinkIndex visitedLinkIndex = visitedLink->getIndex();
       LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
       IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

       if( parentLink == 0 )
       {
           // Preliminary step: find base acceleration
           bufs.dVel[dofDeriv].linksAccelerations(visitedLinkIndex) = -(bufs.aba.linkABIs(visitedLinkIndex).applyInverse(bufs.dVel[dofDeriv].linksBiasWrench(visitedLinkIndex)));

           toEigen(A).block<6,1>(6+model.getNrOfDOFs(),6+model.getNrOfDOFs()+6+dofDeriv) = toEigen(bufs.dVel[dofDeriv].linksAccelerations(visitedLinkIndex));
       }
       else
       {
           LinkIndex    parentLinkIndex = parentLink->getIndex();
           if( toParentJoint->getNrOfDOFs() > 0 )
           {
               size_t dofIndex = toParentJoint->getDOFsOffset();
               assert(toParentJoint->getNrOfDOFs()==1);
               bufs.dVel[dofDeriv].linksAccelerations(visitedLinkIndex) =
                   toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex)*bufs.dVel[dofDeriv].linksAccelerations(parentLinkIndex)
                   + bufs.dVel[dofDeriv].linksBiasAcceleration(visitedLinkIndex);

               double dVel_dofDeriv_ddq_dofIndex = (bufs.dVel[dofDeriv].u(dofIndex)-bufs.aba.U(dofIndex).dot(bufs.dVel[dofDeriv].linksAccelerations(visitedLinkIndex)))/bufs.aba.D(dofIndex);

               A(6+model.getNrOfDOFs()+6+dofIndex,6+model.getNrOfDOFs()+6+dofDeriv) = dVel_dofDeriv_ddq_dofIndex;

               bufs.dVel[dofDeriv].linksAccelerations(visitedLinkIndex) = bufs.dVel[dofDeriv].linksAccelerations(visitedLinkIndex) + bufs.aba.S(dofIndex)*dVel_dofDeriv_ddq_dofIndex;
           }
           else
           {
               //for fixed joints we just propagate the acceleration
               bufs.dVel[dofDeriv].linksAccelerations(visitedLinkIndex) =
                   toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex)*bufs.dVel[dofDeriv].linksAccelerations(parentLinkIndex)
                   + bufs.dVel[dofDeriv].linksBiasAcceleration(visitedLinkIndex);
           }
       }
    }
}

bool ForwardDynamicsLinearization(const Model& model,
                                  const Traversal& traversal,
                                  const FreeFloatingPos& robotPos,
                                  const FreeFloatingVel& robotVel,
                                  const LinkNetExternalWrenches& linkExtWrenches,
                                  const JointDOFsDoubleArray& jointTorques,
                                  ForwardDynamicsLinearizationInternalBuffers & bufs,
                                  FreeFloatingAcc& robotAcc,
                                  FreeFloatingStateLinearization& A)
{
    bool ok = true;
    size_t nDof = model.getNrOfDOFs();

    // First run the ABA algorithm
    ok = ok && ArticulatedBodyAlgorithm(model,traversal,robotPos,robotVel,
                                        linkExtWrenches,jointTorques,bufs.aba,robotAcc);

    // Lets' assume that A is full of zeros
    A.zero();

    // We wrap the matrix with an Eigen::Map to
    // use the Eigen block method to efficiently
    // access the subblock of the matrix
    Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > Aeig = toEigen(A);

    // Then we set the first block column of the linearization matrix, i.e.
    // the left-trivialized derivative with respect to the base position,
    // that is simply
    // \begin{bmatrix}
    //   V_b \times
    //   0
    // - ( \begin{bmatrix} ~^b R_w a_g \end{bmatrix}
    //   0
    // \end{bmatrix}

    //   V_b \times
    Matrix6x6 VbCross = robotVel.baseVel().asCrossProductMatrix();
    Aeig.block(0,0,6,6) = toEigen(VbCross);

    // 0 (not setting becase is done by the global .zero)
    // Aeig.block(6,0,dof,6).setToZero();

    // - ( \begin{bmatrix} ~^b R_w a_g \end{bmatrix}
    // (not setting anything until we have gravity in ABA)

    // 0 (not setting becase is done by the global .zero)
    // Aeig.block(12+dof,0,dof,6).setToZero()

    // Second block-column: the derivative with respect
    // to the joint positions
    //
    // 0
    // 0
    // \partial_{q_J} FD_b
    // \partial_{q_J} FD_J
    //

    // 0 (first two block rows)
    // Aeig.block(0,6,6+nDof,nDof).setToZero()
    //

    // \partial_{q_J} FD_b
    // \partial_{q_J} FD_J
    for(size_t dofDeriv=0; dofDeriv < model.getNrOfDOFs(); dofDeriv++)
    {
        ForwardDynamicsLinearizationWrtJointPos(model,traversal,robotPos,robotVel,
                                                linkExtWrenches,jointTorques,dofDeriv,bufs,robotAcc,A);
    }

    // Third block-column: the derivative with respect
    // to the base velocity
    // I
    // 0
    // \partial_{V_B} FD_b
    // \partial_{V_B} FD_J

    // I
    Aeig.block(0,6+nDof,6,6).setIdentity();

    // 0
    // Aeig.block(6,6+dof,dof,6).setToZero();

    // \partial_{V_B} FD_b
    // \partial_{V_B} FD_J
    ForwardDynamicsLinearizationWrtBaseTwist(model,traversal,robotPos,robotVel,
                                             linkExtWrenches,jointTorques,bufs,robotAcc,A);


    // For

    // Forth block column: the derivative with respect
    // to the joint velocity
    // 0
    // I
    // \partial_{\dot{q}_J} FD_b
    // \partial_{\dot{q}_J} FD_J

    // 0
    //Aeig.block(0,6+dof+6,6,dof).setToZero();

    // I
    Aeig.block(6,6+nDof+6,nDof,nDof).setIdentity();

    // \partial_{\dot{q}_J} FD_b
    // \partial_{\dot{q}_J} FD_J
    for(size_t dofDeriv=0; dofDeriv < model.getNrOfDOFs(); dofDeriv++)
    {
        ForwardDynamicsLinearizationWrtJointVel(model,traversal,robotPos,robotVel,
                                                linkExtWrenches,jointTorques,dofDeriv,bufs,robotAcc,A);
    }

    return ok;
}

}






