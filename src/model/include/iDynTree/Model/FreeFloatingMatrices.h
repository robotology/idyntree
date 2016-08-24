/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_FREE_FLOATING_MATRICES_H
#define IDYNTREE_FREE_FLOATING_MATRICES_H

#include <iDynTree/Core/MatrixDynSize.h>

namespace iDynTree
{

class Model;

/**
 * Enum describing the possible frame velocity representation convention.
 *
 * See KinDynComputations documentation for more details.
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
class FrameJacobian : MatrixDynSize
{
public:
    FrameJacobian(size_t nrOfDofs=0);
    FrameJacobian(const iDynTree::Model & model);

    void resize(const iDynTree::Model & model);
    bool isConsistent(const iDynTree::Model & model) const;

    virtual ~FrameJacobian();
};

/**
 * Jacobian of the 6D momentum.
 */
class MomentumJacobian : MatrixDynSize
{
public:
    MomentumJacobian(size_t nrOfDofs=0);
    MomentumJacobian(const iDynTree::Model & model);

    void resize(const iDynTree::Model & model);
    bool isConsistent(const iDynTree::Model & model) const;

    virtual ~MomentumJacobian();
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
