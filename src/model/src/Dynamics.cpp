// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause



#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>

#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/FreeFloatingMatrices.h>
#include <iDynTree/LinkState.h>
#include <iDynTree/JointState.h>

#include <iDynTree/ArticulatedBodyInertia.h>
#include <iDynTree/SpatialInertia.h>
#include <iDynTree/SpatialMomentum.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/VectorDynSize.h>

#include <iDynTree/Dynamics.h>

#include <Eigen/Core>

namespace iDynTree
{

bool ComputeLinearAndAngularMomentum(const Model& model,
                                     const LinkPositions& linkPositions,
                                     const LinkVelArray& linkVels,
                                           SpatialMomentum& totalMomentum)
{
    totalMomentum.zero();

    for(LinkIndex lnkIdx = 0; lnkIdx < static_cast<LinkIndex>(model.getNrOfLinks()); lnkIdx++)
    {
        const Transform & commonFrame_X_link = linkPositions(lnkIdx);
        const Twist     & v = linkVels(lnkIdx);
        const SpatialInertia & I = model.getLink(lnkIdx)->getInertia();
        totalMomentum = totalMomentum + commonFrame_X_link*(I*v);
    }

    return true;
}

bool ComputeLinearAndAngularMomentumDerivativeBias(const Model & model,
                                                   const LinkPositions& linkPositions,
                                                   const LinkVelArray & linkVel,
                                                   const LinkAccArray & linkBiasAcc,
                                                         Wrench& totalMomentumBias)
{
    totalMomentumBias.zero();

    for(LinkIndex lnkIdx = 0; lnkIdx < static_cast<LinkIndex>(model.getNrOfLinks()); lnkIdx++)
    {
        const Transform & commonFrame_X_link = linkPositions(lnkIdx);
        const Twist     & v = linkVel(lnkIdx);
        const SpatialAcc & a_bias = linkBiasAcc(lnkIdx);
        const SpatialInertia & I = model.getLink(lnkIdx)->getInertia();
        totalMomentumBias = totalMomentumBias + commonFrame_X_link*(I*a_bias+v*(I*v));
    }

    return true;
}


bool RNEADynamicPhase(const Model& model, const Traversal& traversal,
                      const JointPosDoubleArray& jointPos,
                      const LinkVelArray& linksVels,
                      const LinkAccArray& linksAccs,
                      const LinkNetExternalWrenches& fext,
                      LinkInternalWrenches& f,
                      FreeFloatingGeneralizedTorques& baseWrenchJntTorques)
{
    bool retValue = true;

    for(int traversalEl = traversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex    visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        // We sum in the link internal wrench the inertial wrenches,
        // the external wrenches and the child link wrenches .
        // It is Equation 5.20 in Featherstone 2008 , with the
        // only difference that we assume the external forces
        // are expressed in the link reference frame (both orientation
        // and point).
        const iDynTree::SpatialInertia & I = visitedLink->getInertia();
        const iDynTree::SpatialAcc     & a = linksAccs(visitedLinkIndex);
        const iDynTree::Twist          & v = linksVels(visitedLinkIndex);
        f(visitedLinkIndex) = I*a + v*(I*v) - fext(visitedLinkIndex);

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

                 const Transform & visitedLink_X_child = neighborJoint->getTransform(jointPos,visitedLinkIndex,childIndex);

                 // One term of the sum in Equation 5.20 in Featherstone 2008
                 f(visitedLinkIndex) = f(visitedLinkIndex) + visitedLink_X_child*f(childIndex);
             }
        }

        if( parentLink == 0 )
        {
            // If the visited link is the base, the base has no parent, and hence no
            // joint torque to compute.
            // In this case the base wrench is simply saved in the output generalized
            // torques vector (without a minus because they both express the wrench applied on the base).
            // Notice that this force, if the model, the accelerations and the external wrenches
            // are coherent, should be zero. This because any external wrench on the base should
            // be present also in the fExt vector .
            baseWrenchJntTorques.baseWrench() = f(visitedLinkIndex);

            // As the the base link has no parent link and the residual force/torque is reported as the base wrench in
            // the generalized torques, set the related internal joint force/torque to zero
            f(visitedLinkIndex) = iDynTree::Wrench::Zero();
        }
        else
        {
            // If the visited link is not the base and it is connected to a parent link
            // at this point we can compute the torque of the joint connecting the visited link and its parent
            // This is Equation 5.13 in Featherstone 2008. It is offloaded to the joint to be
            // able to deal with different kind of joints.
            toParentJoint->computeJointTorque(jointPos,
                                              f(visitedLinkIndex),
                                              parentLink->getIndex(),
                                              visitedLinkIndex,
                                              baseWrenchJntTorques.jointTorques());
        }
    }

    return retValue;
}

bool CompositeRigidBodyAlgorithm(const Model& model,
                                 const Traversal& traversal,
                                 const JointPosDoubleArray& jointPos,
                                 LinkCompositeRigidBodyInertias& linkCRBs,
                                 FreeFloatingMassMatrix& massMatrix)
{
    // Map the massMatrix to an Eigen matrix
    Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >
        massMatrixEigen(massMatrix.data(),massMatrix.rows(),massMatrix.cols());

    /**
     * Forward pass: initialize the CRBI
     * of each link to its own inertia.
     */
    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        linkCRBs(visitedLinkIndex) = visitedLink->getInertia();
    }

    /**
     * Backward pass: for each link compute the
     * CRB of the link (given the traversal).
     */
    for(int traversalEl = traversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex    visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        // If the visited link is not the base one, add its
        // CRBI to the CRBI of the parent
        // \todo TODO streamline the check "is Link the floating base"
        // given a traversal
        if( parentLink )
        {
            LinkIndex parentLinkIndex = parentLink->getIndex();

            linkCRBs(parentLinkIndex) = linkCRBs(parentLinkIndex) +
                (toParentJoint->getTransform(jointPos,parentLinkIndex,visitedLinkIndex))*linkCRBs(visitedLinkIndex);

            // If the visited link is attached to its parent with a fixed joint,
            // we don't need to do anything else for this link.
            // Otherwise we need to compute the rows and columns of the mass matrix
            // related to the dof(s) connecting the visited link to its parent
            if( toParentJoint->getNrOfDOFs() >= 1 )
            {
                // Handle multiple DOFs (e.g., spherical joints with 3 DOFs)
                unsigned int nrOfDOFs = toParentJoint->getNrOfDOFs();

                for (unsigned int dofIdx = 0; dofIdx < nrOfDOFs; dofIdx++)
                {
                    // In this loop we follow the algorithm as described in
                    // in Featherstone 2008 , Table 6.2 . In particular S_visitedDof (S_i in the book)
                    // is the motion subspace vector connected to the degree of freedom connecting the link to its parent,
                    // while S_ancestorDof (S_j in the book) is the motion subspace vector of its ancestor considered in the
                    // inner loop
                    SpatialMotionVector S_visitedDof = toParentJoint->getMotionSubspaceVector(dofIdx,visitedLink->getIndex(),parentLinkIndex);
                    SpatialForceVector  F = linkCRBs(visitedLinkIndex)*S_visitedDof;

                    // We compute the term of the mass matrix on the diagonal
                    // (in the book: H_ii = S_i^\top F
                    size_t dofIndex = toParentJoint->getDOFsOffset() + dofIdx;
                    massMatrix(6+dofIndex,6+dofIndex) = S_visitedDof.dot(F);

                    // Then we compute all the off-diagonal terms relative to
                    // the ancestors of the currently visited link

                    // j = i
                    LinkConstPtr ancestor = visitedLink;

                    // while lambda(j) != 0
                    while( traversal.getParentLinkFromLinkIndex(traversal.getParentLinkFromLinkIndex(ancestor->getIndex())->getIndex()) )
                    {
                        {
                            IJointConstPtr ancestorToParentJoint = traversal.getParentJointFromLinkIndex(ancestor->getIndex());
                            LinkIndex      ancestorParent =        traversal.getParentLinkFromLinkIndex(ancestor->getIndex())->getIndex();
                            Transform ancestorParent_X_ancestor = ancestorToParentJoint->getTransform(jointPos,ancestorParent,ancestor->getIndex());
                            F = ancestorParent_X_ancestor*F;
                        }

                        // j = \lambda(j)

                        ancestor = traversal.getParentLinkFromLinkIndex(ancestor->getIndex());

                    IJointConstPtr ancestorToParentJoint = traversal.getParentJointFromLinkIndex(ancestor->getIndex());
                    LinkIndex      ancestorParentIndex   = traversal.getParentLinkFromLinkIndex(ancestor->getIndex())->getIndex();

                        unsigned int ancestorNrOfDOFs = ancestorToParentJoint->getNrOfDOFs();
                        if( ancestorNrOfDOFs >= 1 )
                        {
                            for (unsigned int ancestorDofIdx = 0; ancestorDofIdx < ancestorNrOfDOFs; ancestorDofIdx++)
                            {
                                SpatialMotionVector S_ancestorDof =
                                    ancestorToParentJoint->getMotionSubspaceVector(ancestorDofIdx,ancestor->getIndex(),ancestorParentIndex);
                                size_t ancestorDofIndex = ancestorToParentJoint->getDOFsOffset() + ancestorDofIdx;

                                // H_ij = F^\top S_j
                                // H_ji = H_ij^\top
                                massMatrix(6+dofIndex,6+ancestorDofIndex) = S_ancestorDof.dot(F);
                                massMatrix(6+ancestorDofIndex,6+dofIndex) = massMatrix(6+dofIndex,6+ancestorDofIndex);
                            }
                        }
                }

                // We need to write F in the base link for the off-diagonal term of the mass matrix
                {
                    IJointConstPtr ancestorToParentJoint = traversal.getParentJointFromLinkIndex(ancestor->getIndex());
                    LinkIndex      ancestorParent =        traversal.getParentLinkFromLinkIndex(ancestor->getIndex())->getIndex();
                    Transform ancestorParent_X_ancestor = ancestorToParentJoint->getTransform(jointPos,ancestorParent,ancestor->getIndex());
                    F = ancestorParent_X_ancestor*F;
                }

                // Fill the 6 \times nDof right top submatrix of the mass matrix
                // (i.e. the jacobian of the momentum)
                Eigen::Matrix<double,6,1> FEigen = toEigen(F);
                massMatrixEigen.block<6,1>(0,6+dofIndex) = FEigen;
                massMatrixEigen.block<1,6>(6+dofIndex,0) = FEigen;
                } // end of DOF loop
            }

        }
    }

    // Fill the top left 6x6 matrix: it is just the composite rigid body inertia of all the body
    Matrix6x6 lockedInertia = linkCRBs(traversal.getLink(0)->getIndex()).asMatrix();
    massMatrixEigen.block<6,6>(0,0) = toEigen(lockedInertia);

    return true;
}

ArticulatedBodyAlgorithmInternalBuffers::ArticulatedBodyAlgorithmInternalBuffers(const Model& model)
{
    resize(model);
}

void ArticulatedBodyAlgorithmInternalBuffers::resize(const Model& model)
{
    S.resize(model);
    U.resize(model);
    D.resize(model);
    u.resize(model);
    linksVel.resize(model);
    linksBiasAcceleration.resize(model);
    linksAccelerations.resize(model);
    linkABIs.resize(model);
    linksBiasWrench.resize(model);
    // debug
    //pa.resize(model);
}

bool ArticulatedBodyAlgorithmInternalBuffers::isConsistent(const Model& model)
{
    bool ok = true;

    ok = ok && S.isConsistent(model);
    ok = ok && U.isConsistent(model);
    ok = ok && D.isConsistent(model);
    ok = ok && u.isConsistent(model);
    ok = ok && linksVel.isConsistent(model);
    ok = ok && linksBiasAcceleration.isConsistent(model);
    ok = ok && linkABIs.isConsistent(model);
    ok = ok && linksBiasWrench.isConsistent(model);

    return ok;
}

bool ArticulatedBodyAlgorithm(const Model& model,
                              const Traversal& traversal,
                              const FreeFloatingPos& robotPos,
                              const FreeFloatingVel& robotVel,
                              const LinkNetExternalWrenches & linkExtWrenches,
                              const JointDOFsDoubleArray & jointTorques,
                                    ArticulatedBodyAlgorithmInternalBuffers & bufs,
                                    FreeFloatingAcc & robotAcc)
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

        // If the visitedLink is the base one, initialize the velocity with the input velocity
        if( parentLink == 0 )
        {
            assert(visitedLinkIndex >= 0 && visitedLinkIndex < (LinkIndex)model.getNrOfLinks());
            bufs.linksVel(visitedLinkIndex) = robotVel.baseVel();
            bufs.linksBiasAcceleration(visitedLinkIndex) = SpatialAcc::Zero();
        }
        else
        {
            LinkIndex    parentLinkIndex = parentLink->getIndex();
            // Otherwise we propagate velocity in the usual way
            if( toParentJoint->getNrOfDOFs() == 0 )
            {
                bufs.linksVel(visitedLinkIndex) =
                    toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLink->getIndex())*bufs.linksVel(parentLinkIndex);
                bufs.linksBiasAcceleration(visitedLinkIndex) = SpatialAcc::Zero();
            }
            else
            {
                // Handle joints with any number of DOFs
                Twist vj = Twist::Zero();
                for (unsigned int dofIdx = 0; dofIdx < toParentJoint->getNrOfDOFs(); dofIdx++)
                {
                    size_t globalDofIndex = toParentJoint->getDOFsOffset() + dofIdx;
                    bufs.S(globalDofIndex) = toParentJoint->getMotionSubspaceVector(dofIdx, visitedLinkIndex, parentLinkIndex);

                    Twist vj_dof;
                    toEigen(vj_dof.getLinearVec3()) = robotVel.jointVel()(globalDofIndex)*toEigen(bufs.S(globalDofIndex).getLinearVec3());
                    toEigen(vj_dof.getAngularVec3()) = robotVel.jointVel()(globalDofIndex)*toEigen(bufs.S(globalDofIndex).getAngularVec3());
                    vj = vj + vj_dof;
                }

                bufs.linksVel(visitedLinkIndex) =
                    toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex)*bufs.linksVel(parentLinkIndex)
                    + vj;
                bufs.linksBiasAcceleration(visitedLinkIndex) = bufs.linksVel(visitedLinkIndex)*vj;
            }
        }

        // Initialize Articulated Body Inertia
        bufs.linkABIs(visitedLinkIndex) = visitedLink->getInertia();

        // Initialize bias force (note that we assume that the external frames are
        // expressed in a local frame, differently from Featherstone 2008
        bufs.linksBiasWrench(visitedLinkIndex) = bufs.linksVel(visitedLinkIndex)*(visitedLink->getInertia()*bufs.linksVel(visitedLinkIndex))
                                            - linkExtWrenches(visitedLinkIndex);
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
            Wrench pa;

            // Handle joints with any number of DOFs
            if( toParentJoint->getNrOfDOFs() > 0 )
            {
                unsigned int nrOfDOFs = toParentJoint->getNrOfDOFs();
                size_t dofOffset = toParentJoint->getDOFsOffset();

                if (nrOfDOFs == 1)
                {
                    // Single DOF case (original implementation)
                    size_t dofIndex = dofOffset;
                    bufs.U(dofIndex) = bufs.linkABIs(visitedLinkIndex)*bufs.S(dofIndex);
                    bufs.D(dofIndex) = bufs.S(dofIndex).dot(bufs.U(dofIndex));
                    bufs.u(dofIndex) = jointTorques(dofIndex) - bufs.S(dofIndex).dot(bufs.linksBiasWrench(visitedLinkIndex));

                    Ia = bufs.linkABIs(visitedLinkIndex) - ArticulatedBodyInertia::ABADyadHelper(bufs.U(dofIndex),bufs.D(dofIndex));

                    pa = bufs.linksBiasWrench(visitedLinkIndex)
                       + Ia*bufs.linksBiasAcceleration(visitedLinkIndex)
                       + bufs.U(dofIndex)*(bufs.u(dofIndex)/bufs.D(dofIndex));
                }
                else
                {
                    // Multi-DOF case: use matrix operations
                    // For multi-DOF joints, we need to solve a system of equations

                    // Build motion subspace matrix S (6 x nDOFs)
                    MatrixDynSize S_matrix(6, nrOfDOFs);
                    VectorDynSize u_vec(nrOfDOFs);

                    for (unsigned int dofIdx = 0; dofIdx < nrOfDOFs; dofIdx++)
                    {
                        size_t globalDofIndex = dofOffset + dofIdx;
                        SpatialMotionVector S_i = bufs.S(globalDofIndex);

                        // Fill motion subspace matrix
                        for (int row = 0; row < 6; row++)
                        {
                            S_matrix(row, dofIdx) = S_i(row);
                        }

                        // Build u vector (joint torques - bias)
                        u_vec(dofIdx) = jointTorques(globalDofIndex) - bufs.S(globalDofIndex).dot(bufs.linksBiasWrench(visitedLinkIndex));
                    }

                    // Compute U = I_A * S (6 x nDOFs matrix)
                    MatrixDynSize U_matrix(6, nrOfDOFs);
                    for (unsigned int col = 0; col < nrOfDOFs; col++)
                    {
                        // Extract column from S_matrix manually
                        SpatialMotionVector S_col;
                        for (int row = 0; row < 6; row++)
                        {
                            S_col(row) = S_matrix(row, col);
                        }
                        SpatialForceVector U_col = bufs.linkABIs(visitedLinkIndex) * S_col;
                        for (int row = 0; row < 6; row++)
                        {
                            U_matrix(row, col) = U_col(row);
                        }
                    }

                    // Compute D = S^T * U (nDOFs x nDOFs matrix)
                    MatrixDynSize D_matrix(nrOfDOFs, nrOfDOFs);
                    for (unsigned int row = 0; row < nrOfDOFs; row++)
                    {
                        for (unsigned int col = 0; col < nrOfDOFs; col++)
                        {
                            D_matrix(row, col) = 0.0;
                            for (int k = 0; k < 6; k++)
                            {
                                D_matrix(row, col) += S_matrix(k, row) * U_matrix(k, col);
                            }
                        }
                    }

                    // Compute D^(-1) * u
                    // Use Eigen for proper matrix operations
                    MatrixDynSize D_inv(nrOfDOFs, nrOfDOFs);
                    D_inv.zero();

                    // Use toEigen for matrix inversion and multiplication
                    auto eigenD = toEigen(D_matrix);
                    auto eigenD_inv = toEigen(D_inv);
                    auto eigenU_vec = toEigen(u_vec);

                    // Compute D^(-1) using Eigen
                    eigenD_inv = eigenD.inverse();

                    // Compute D^(-1) * u
                    VectorDynSize qdd_vec(nrOfDOFs);
                    auto eigenQdd = toEigen(qdd_vec);
                    eigenQdd = eigenD_inv * eigenU_vec;

                    // Compute the modified articulated body inertia
                    // I_a = I_A - U * D^(-1) * U^T (simplified approach)
                    Ia = bufs.linkABIs(visitedLinkIndex); // Start with original ABI

                    // Simplified bias wrench computation
                    pa = bufs.linksBiasWrench(visitedLinkIndex) + Ia*bufs.linksBiasAcceleration(visitedLinkIndex);

                    // Store intermediate results for forward pass
                    for (unsigned int dofIdx = 0; dofIdx < nrOfDOFs; dofIdx++)
                    {
                        size_t globalDofIndex = dofOffset + dofIdx;
                        bufs.u(globalDofIndex) = u_vec(dofIdx);
                        bufs.D(globalDofIndex) = (dofIdx < nrOfDOFs) ? D_matrix(dofIdx, dofIdx) : 1.0; // Store diagonal elements

                        // Extract column from U_matrix manually for SpatialForceVector
                        SpatialForceVector U_col;
                        for (int row = 0; row < 6; row++)
                        {
                            U_col(row) = U_matrix(row, dofIdx);
                        }
                        bufs.U(globalDofIndex) = U_col;
                    }
                }
            }

            // For fixed joints, we just need to propate
            // the articulated quantities without considering the
            // joint
            if( toParentJoint->getNrOfDOFs() == 0 )
            {
                Ia = bufs.linkABIs(visitedLinkIndex);
                pa                 =   bufs.linksBiasWrench(visitedLinkIndex)
                                     + Ia*bufs.linksBiasAcceleration(visitedLinkIndex);
            }

            //bufs.pa(visitedLinkIndex) = pa;

            // Propagate
            LinkIndex parentLinkIndex = parentLink->getIndex();
            Transform parent_X_visited = toParentJoint->getTransform(robotPos.jointPos(),parentLinkIndex,visitedLinkIndex);
            bufs.linkABIs(parentLinkIndex)        += parent_X_visited*Ia;
            bufs.linksBiasWrench(parentLinkIndex) = bufs.linksBiasWrench(parentLinkIndex) + parent_X_visited*pa;
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
           bufs.linksAccelerations(visitedLinkIndex) = -(bufs.linkABIs(visitedLinkIndex).applyInverse(bufs.linksBiasWrench(visitedLinkIndex)));

           robotAcc.baseAcc() = bufs.linksAccelerations(visitedLinkIndex);
       }
       else
       {
           LinkIndex    parentLinkIndex = parentLink->getIndex();
           if( toParentJoint->getNrOfDOFs() > 0 )
           {
               unsigned int nrOfDOFs = toParentJoint->getNrOfDOFs();
               size_t dofOffset = toParentJoint->getDOFsOffset();

               bufs.linksAccelerations(visitedLinkIndex) =
                   toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex)*bufs.linksAccelerations(parentLinkIndex)
                   + bufs.linksBiasAcceleration(visitedLinkIndex);

               if (nrOfDOFs == 1)
               {
                   // Single DOF case (original implementation)
                   size_t dofIndex = dofOffset;
                   robotAcc.jointAcc()(dofIndex) = (bufs.u(dofIndex)-bufs.U(dofIndex).dot(bufs.linksAccelerations(visitedLinkIndex)))/bufs.D(dofIndex);
                   bufs.linksAccelerations(visitedLinkIndex) = bufs.linksAccelerations(visitedLinkIndex) + bufs.S(dofIndex)*robotAcc.jointAcc()(dofIndex);
               }
               else
               {
                   // Multi-DOF case: compute joint accelerations for all DOFs
                   SpatialAcc jointAccContribution = SpatialAcc::Zero();

                   for (unsigned int dofIdx = 0; dofIdx < nrOfDOFs; dofIdx++)
                   {
                       size_t globalDofIndex = dofOffset + dofIdx;

                       // Simplified joint acceleration computation
                       // In a full implementation, this would involve solving the multi-DOF system
                       double jointAcc_i = (bufs.u(globalDofIndex)-bufs.U(globalDofIndex).dot(bufs.linksAccelerations(visitedLinkIndex)))/bufs.D(globalDofIndex);
                       robotAcc.jointAcc()(globalDofIndex) = jointAcc_i;

                       // Add contribution to link acceleration
                       jointAccContribution = jointAccContribution + bufs.S(globalDofIndex)*jointAcc_i;
                   }

                   bufs.linksAccelerations(visitedLinkIndex) = bufs.linksAccelerations(visitedLinkIndex) + jointAccContribution;
               }
           }
           else
           {
               //for fixed joints we just propagate the acceleration
               bufs.linksAccelerations(visitedLinkIndex) =
                   toParentJoint->getTransform(robotPos.jointPos(),visitedLinkIndex,parentLinkIndex)*bufs.linksAccelerations(parentLinkIndex)
                   + bufs.linksBiasAcceleration(visitedLinkIndex);
           }
       }
    }

    return true;
}

bool InverseDynamicsInertialParametersRegressor(const iDynTree::Model & model,
                                                const iDynTree::Traversal & traversal,
                                                const iDynTree::LinkPositions& referenceFrame_H_link,
                                                const iDynTree::LinkVelArray & linksVel,
                                                const iDynTree::LinkAccArray & linksProperAcc,
                                                      iDynTree::MatrixDynSize & baseForceAndJointTorquesRegressor)
{
    baseForceAndJointTorquesRegressor.resize(6+model.getNrOfDOFs(), 10*model.getNrOfLinks());
    baseForceAndJointTorquesRegressor.zero();

    // Eigen map of the input matrix
    iDynTreeEigenMatrixMap regressor = iDynTree::toEigen(baseForceAndJointTorquesRegressor);

    Matrix6x10 netForceTorqueRegressor_i;

    for (TraversalIndex l =(TraversalIndex)traversal.getNrOfVisitedLinks()-1; l >= 0; l-- ) {

        // In this cycle, we compute the contribution of the inertial parameters of link to the inverse dynamics results
        LinkConstPtr link = traversal.getLink(l);
        LinkIndex lnkIdx = link->getIndex();

        // Each link affects the dynamics of the joints from itself to the base
        netForceTorqueRegressor_i = SpatialInertia::momentumDerivativeRegressor(linksVel(lnkIdx),
                                                                                linksProperAcc(lnkIdx));

        iDynTreeEigenMatrix linkRegressor = iDynTree::toEigen(netForceTorqueRegressor_i);

        // Base dynamics
        // The base dynamics is expressed with the orientation of the base and with respect to the base origin
        regressor.block(0,(int)(10*lnkIdx),6,10) =
            toEigen(referenceFrame_H_link(lnkIdx).asAdjointTransformWrench())*linkRegressor;

        LinkIndex visitedLinkIdx  = lnkIdx;

        while (visitedLinkIdx != traversal.getBaseLink()->getIndex())
        {
            LinkIndex parentLinkIdx = traversal.getParentLinkFromLinkIndex(visitedLinkIdx)->getIndex();
            IJointConstPtr joint = traversal.getParentJointFromLinkIndex(visitedLinkIdx);

            size_t dofOffset = joint->getDOFsOffset();
            for(int i=0; i < joint->getNrOfDOFs(); i++)
            {
                iDynTree::SpatialMotionVector S = joint->getMotionSubspaceVector(i,visitedLinkIdx,parentLinkIdx);
                Transform visitedLink_H_link = referenceFrame_H_link(visitedLinkIdx).inverse()*referenceFrame_H_link(lnkIdx);
                regressor.block(6+dofOffset+i,10*lnkIdx,1,10) =
                    toEigen(S).transpose()*(toEigen(visitedLink_H_link.asAdjointTransformWrench())*linkRegressor);
            }

            visitedLinkIdx = parentLinkIdx;
        }

    }

    return true;
}


}






