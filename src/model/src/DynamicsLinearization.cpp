/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Model/DynamicsLinearization.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>

#include <iDynTree/Model/FreeFloatingState.h>

#include <iDynTree/Core/EigenHelpers.h>

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
                                             const LinkExternalWrenches& linkExtWrenches,
                                             const JointDoubleArray& jointTorques,
                                             ForwardDynamicsLinearizationInternalBuffers & bufs,
                                             FreeFloatingAcc& robotAcc,
                                             FreeFloatingStateLinearization& A);

void ForwardDynamicsLinearizationWrtBaseTwist(const Model& model,
                                              const Traversal& traversal,
                                              const FreeFloatingPos& robotPos,
                                              const FreeFloatingVel& robotVel,
                                              const LinkExternalWrenches& linkExtWrenches,
                                              const JointDoubleArray& jointTorques,
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

        // We fill the buffer of the derivative of biasWrench with respect to link velocity
        // that will be used by everyone
       const iDynTree::SpatialInertia & M = visitedLink->getInertia();
       toEigen(bufs.dVl_linkLocalBiasWrench[visitedLinkIndex]) = toEigen(M.biasWrenchDerivative(bufs.aba.linksVel(visitedLinkIndex)));
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
                                             const LinkExternalWrenches& linkExtWrenches,
                                             const JointDoubleArray& jointTorques,
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

        // Initialize bias force derivative
        const iDynTree::SpatialInertia & M = visitedLink->getInertia();
        toEigen(bufs.dVl_linkLocalBiasWrench[visitedLinkIndex]) = toEigen(M.biasWrenchDerivative(bufs.aba.linksVel(visitedLinkIndex)));
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
                                  const LinkExternalWrenches& linkExtWrenches,
                                  const JointDoubleArray& jointTorques,
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
    for(size_t h = 0; h < nDof; h++)
    {

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






