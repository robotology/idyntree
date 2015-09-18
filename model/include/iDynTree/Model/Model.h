/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_MODEL_H
#define IDYNTREE_MODEL_H

#include <iDynTree/Model/IJoint.h>
#include <iDynTree/Model/Link.h>

#include <iDynTree/Model/Indeces.h>

#include <vector>

namespace iDynTree
{
    class Traversal;

    struct Neighbor
    {
        LinkPtr neighborLink;
        IJointPtr neighborJoint;
    };

    /**
     * Class that represents a generic multibody model.
     *
     *
     * \ingroup iDynTreeModel
     */
    class Model
    {
    private:
        /** Vector of links. For each link its index indicates its location in this vector */
        std::vector<Link> links;

        /** Vector of joints. For each joint its index indicates its location in this vector */
        std::vector<IJointPtr> joints;

        /** Vector of link names, matches the index of each link to its name. */
        std::vector<std::string> linkNames;

        /** Vector of joint names, matches the index of each joint to its name. */
        std::vector<std::string> jointNames;

        /** Adjacency lists: match each link index to a list of its neighbors,
            and the joint connecting to them. */
        std::vector< std::vector<Neighbor> > neighbors;

        /**
         * Most data structures are not undirected, so we store the original
         * root of the tree, to provide a default root for Traversal generation.
         */
        LinkIndex defaultBaseLink;

        /**
         * Check if a name is already used for a link in the model.
         *
         * @return true if a name is used by a link in a model, false otherwise.
         */
        bool isLinkNameUsed(const std::string linkName);

        /**
         * Check if a name is already used for a joint in the model.
         *
         * @return true if a name is used by a joint in a model, false otherwise.
         */
        bool isJointNameUsed(const std::string jointName);

        /**
         * Copy the structure of the model from another instance of a model.
         */
        void copy(const Model & model);

        /**
         * Destroy the object, properly deallocating memory.
         */
        void destroy();

    public:

        /**
         * Costructor
         */
        Model();

        /**
         * Copy costructor
         */
        Model(const Model & other);

        /**
         * Copy operator
         */
        Model& operator=(const Model &other);

        /**
         * Destructor
         *
         */
        virtual ~Model();

        /**
         * Get the number of links in the model.
         */
        int getNrOfLinks() const;

        /**
         * Get the name of a link given its index, or
         * an LINK_INVALID_NAME string if linkIndex < 0 or >= getNrOfLinks()
         */
        std::string  getLinkName(const LinkIndex linkIndex) const;

        LinkIndex getLinkIndex(const std::string & linkName) const;

        LinkPtr getLink(const LinkIndex linkIndex);
        LinkConstPtr getLink(const LinkIndex linkIndex) const;

        LinkIndex addLink(const std::string & name, const Link & link);

        /**
         * Get number of joints in the model.
         */
        int getNrOfJoints() const;

        /**
         * Get the name of a link given its index, or
         * an JOINT_INVALID_NAME if linkIndex < 0 or >= getNrOfLinks()
         */
        std::string getJointName(const JointIndex index) const;

        /**
         * Get the index of a joint, given a jointName.
         * If the jointName is not found in the model,
         * return JOINT_INVALID_INDEX .
         *
         */
        JointIndex getJointIndex(const std::string & jointName) const;

        IJointPtr getJoint(const JointIndex index);

        IJointConstPtr getJoint(const JointIndex index) const;

        /**
         *
         * @return the JointIndex of the added joint, or JOINT_INVALID_INDEX if
         *         there was an error in adding the joint.
         */
        JointIndex addJoint(const std::string & jointName, IJointConstPtr joint);

        /**
         * Get the nr of neighbors of a given link.
         */
        unsigned int getNrOfNeighbors(const LinkIndex link) const;

        /**
         * Get the neighbor of a link. neighborIndex should be
         * >= 0 and <= getNrOfNeighbors(link)
         */
        Neighbor getNeighbor(const LinkIndex link, unsigned int neighborIndex) const;

        /**
         * Set the default base link, used for generation of the default traversal.
         */
        bool setDefaultBaseLink(const LinkIndex linkIndex);

        /**
         * Get the default base link, used for generation of the default traversal.
         */
        LinkIndex getDefaultBaseLink() const;

        /**
         * Compute a Traversal of all the links in the Model, doing a Depth First Search starting
         * at the default base.
         *
         * \warning this function works only on Models without cycles
         */
        bool computeFullTreeTraversal(Traversal & traversal);

       /**
         * Compute a Traversal of all the links in the Model, doing a Depth First Search starting
         * at the given traversalBase.
         *
         * \warning this function works only on Models without cycles
         */
        bool computeFullTreeTraversal(Traversal & traversal, const LinkIndex traversalBase);

    };


}

#endif /* IDYNTREE_LINK_H */