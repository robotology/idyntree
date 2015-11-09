/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_JOINT_STATE_H
#define IDYNTREE_JOINT_STATE_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/SpatialForceVector.h>

#include <vector>

namespace iDynTree
{
    class Model;

    /**
     * Class for storing a vector of scalar values,
     *  one for each dof in a model.
     */
    class DOFDoubleArray: public VectorDynSize
    {
    public:
        DOFDoubleArray(unsigned int nrOfDOFs = 0);
        DOFDoubleArray(const iDynTree::Model & model);

        void resize(unsigned int nrOfDOFs);
        void resize(const iDynTree::Model & model);

        ~DOFDoubleArray();
    };

    /**
     * Class for storing a vector of spatial force vectors,
     *  one for each dof in a model.
     */

    /**
     * Class for storing a vector of spatial accelerations,
     *  one for each link in a model.
     */
    class DOFSpatialForceArray
    {
    private:
        std::vector<iDynTree::SpatialForceVector> m_dofSpatialForce;

    public:
        DOFSpatialForceArray(unsigned int nrOfDOFs = 0);
        DOFSpatialForceArray(const iDynTree::Model & model);

        void resize(const unsigned int nrOfDOFs);
        void resize(const iDynTree::Model & model);

        iDynTree::SpatialForceVector & operator()(const size_t dof);
        const iDynTree::SpatialForceVector & operator()(const size_t dof) const;

        ~DOFSpatialForceArray();
    };
}

#endif /* IDYNTREE_JOINT_STATE_H */