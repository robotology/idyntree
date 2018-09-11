/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_FREE_FLOATING_MATRICES_H
#define IDYNTREE_FREE_FLOATING_MATRICES_H

#include <iDynTree/Core/MatrixDynSize.h>

namespace iDynTree
{

class Model;

/**
 * @brief Possible frame velocity representation convention.
 *
 * Given a link \f$L\f$ and an absolute frame \f$A\f$, the
 * the possible frame velocity representation are the following:
 * * `INERTIAL_FIXED_REPRESENTATION` : Velocity representation is \f${}^{A} \mathrm{v}_{A,B}\f$,
 * * `BODY_FIXED_REPRESENTATION` : Velocity representation is \f${}^{B} \mathrm{v}_{A,B}\f$,
 * * `MIXED_REPRESENTATION` : Velocity representation is \f${}^{B[A]} \mathrm{v}_{A,B}\f$.
 *
 * See iDynTree::KinDynComputations documentation for more details.
 */
enum FrameVelocityRepresentation
{
    INERTIAL_FIXED_REPRESENTATION,
    BODY_FIXED_REPRESENTATION,
    MIXED_REPRESENTATION
};

/**
 * Jacobian of the 6D frame velocity.
 */
class FrameFreeFloatingJacobian : public MatrixDynSize
{
public:
    FrameFreeFloatingJacobian(size_t nrOfDofs=0);
    FrameFreeFloatingJacobian(const iDynTree::Model & model);

    void resize(const iDynTree::Model & model);
    bool isConsistent(const iDynTree::Model & model) const;

    virtual ~FrameFreeFloatingJacobian();
};

/**
 * Jacobian of the 6D momentum.
 */
class MomentumFreeFloatingJacobian : public MatrixDynSize
{
public:
    MomentumFreeFloatingJacobian(size_t nrOfDofs=0);
    MomentumFreeFloatingJacobian(const iDynTree::Model & model);

    void resize(const iDynTree::Model & model);
    bool isConsistent(const iDynTree::Model & model) const;

    virtual ~MomentumFreeFloatingJacobian();
};


/**
 * Class representing the mass matrix of a Free Floating robot.
 *
 *
 */
class FreeFloatingMassMatrix : public MatrixDynSize
{
public:
    FreeFloatingMassMatrix(size_t nrOfDofs=0);

    /**
     * Constructor from a model, to get the appropriate size of the
     * mass matrix.
     */
    FreeFloatingMassMatrix(const iDynTree::Model& model);

    /**
     * Resize the class to match the dimension of the dofs contained in a Model.
     *
     * @param model the model from which to get the number and dimension of the joints.
     *
     * @warning This method wipes the content of the mass matrix.
     *
     */
    void resize(const iDynTree::Model& model);


    /**
      * Destructor
      */
    virtual ~FreeFloatingMassMatrix();
};

}

#endif
