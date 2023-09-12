// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_JOINT_STATE_H
#define IDYNTREE_JOINT_STATE_H

#include <iDynTree/VectorFixSize.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/SpatialForceVector.h>

#include <vector>

namespace iDynTree
{
    class Model;
    class SpatialMotionVector;

    /**
     * Class for storing a vector of scalar values,
     *  one for each internal position coordinate in a model.
     */
    class JointPosDoubleArray: public VectorDynSize
    {
    public:
        JointPosDoubleArray(std::size_t nrOfDOFs = 0);
        JointPosDoubleArray(const iDynTree::Model & model);

        void resize(std::size_t nrOfDOFs);
        void resize(const iDynTree::Model & model);

        bool isConsistent(const iDynTree::Model & model) const;
        JointPosDoubleArray& operator=(const iDynTree::VectorDynSize& input);

        ~JointPosDoubleArray();
    };

    /**
     * Class for storing a vector of scalar values,
     *  one for each internal coordinate in a model.
     */
    class JointDOFsDoubleArray: public VectorDynSize
    {
    public:
        JointDOFsDoubleArray(std::size_t nrOfDOFs = 0);
        JointDOFsDoubleArray(const iDynTree::Model & model);

        void resize(std::size_t nrOfDOFs);
        void resize(const iDynTree::Model & model);

        bool isConsistent(const iDynTree::Model & model) const;
        JointDOFsDoubleArray& operator=(const iDynTree::VectorDynSize& input);

        ~JointDOFsDoubleArray();
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
        DOFSpatialForceArray(std::size_t nrOfDOFs = 0);
        DOFSpatialForceArray(const iDynTree::Model & model);

        void resize(const std::size_t nrOfDOFs);
        void resize(const iDynTree::Model & model);

        bool isConsistent(const iDynTree::Model & model) const;

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
        DOFSpatialMotionArray(std::size_t nrOfDOFs = 0);
        DOFSpatialMotionArray(const iDynTree::Model & model);

        void resize(const std::size_t nrOfDOFs);
        void resize(const iDynTree::Model & model);

        bool isConsistent(const iDynTree::Model & model) const;

        iDynTree::SpatialMotionVector & operator()(const size_t dof);
        const iDynTree::SpatialMotionVector & operator()(const size_t dof) const;

        ~DOFSpatialMotionArray();
    };
}

#endif
