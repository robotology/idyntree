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
    class SpatialMotionVector;

    /**
     * Class for storing a vector of scalar values,
     *  one for each dof in a model.
     */
    class JointDoubleArray: public VectorDynSize
    {
    public:
        JointDoubleArray(unsigned int nrOfDOFs = 0);
        JointDoubleArray(const iDynTree::Model & model);

        void resize(unsigned int nrOfDOFs);
        void resize(const iDynTree::Model & model);

        ~JointDoubleArray();
    };

    /**
     * Class for storing a vector of spatial force vectors,
     *  one for each dof in a model.
     */

    /**
     * Class for storing a vector of spatial force vectors,
     *  one for each (internal) dof in a model.
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

    /**
     * Class for storing a vector of spatial motion vectors,
     *  one for each (internal) dof in a model.
     */
    class DOFSpatialMotionArray
    {
    private:
        std::vector<iDynTree::SpatialMotionVector> m_dofSpatialMotion;

    public:
        DOFSpatialMotionArray(unsigned int nrOfDOFs = 0);
        DOFSpatialMotionArray(const iDynTree::Model & model);

        void resize(const unsigned int nrOfDOFs);
        void resize(const iDynTree::Model & model);

        iDynTree::SpatialMotionVector & operator()(const size_t dof);
        const iDynTree::SpatialMotionVector & operator()(const size_t dof) const;

        ~DOFSpatialMotionArray();
    };
}

#endif /* IDYNTREE_JOINT_STATE_H */