/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_FREE_FLOATING_MASS_MATRIX_H
#define IDYNTREE_FREE_FLOATING_MASS_MATRIX_H

#include <iDynTree/Core/MatrixDynSize.h>

namespace iDynTree
{
    class Model;


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
         * Resize the class to match the dimension of the joints contained in a Model.
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

#endif /* IDYNTREE_FREE_FLOATING_STATE_H */