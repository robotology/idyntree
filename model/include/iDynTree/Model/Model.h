/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_MODEL_H
#define IDYNTREE_MODEL_H

#include <iDynTree/Model/Link.h>

#include <iDynTree/Model/Indeces.h>

#include <vector>

namespace iDynTree
{
    class IJoint;

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
        std::vector<IJoint *> joints;

        /** Vector of link names, matches the index of each link to its name. */
        std::vector<std::string> linkNames;

        /** Vector of joint names, matches the index of each joint to its name. */
        std::vector<std::string> jointNames;

        struct neighbor {
            Link * link;
            IJoint * joint;
        };

        /** Adjacency lists: match each link index to a list of its neighbors,
            and the joint connecting to them. */
        std::vector<neighbor> neighbors;


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
         * an empty string if linkIndex < 0 or >= getNrOfLinks()
         */
        std::string  getLinkName(const LinkIndex linkIndex) const;

        LinkIndex getLinkIndex(const std::string & linkName) const;

        Link * getLink(const LinkIndex linkIndex);
        const Link * getLink(const LinkIndex linkIndex) const;

        LinkIndex addLink(const std::string & name, const Link & link);

        /**
         * Get number of joints in the model.
         */
        int getNrOfJoints() const;

        /**
         * Get the name of a link given its index, or
         * an empty string if linkIndex < 0 or >= getNrOfLinks()
         */
        std::string getJointName(const JointIndex index) const;

        /**
         * Get the index of a joint, given a jointName.
         * If the jointName is not found in the model,
         * return JOINT_INVALID_INDEX .
         *
         */
        JointIndex getJointIndex(const std::string & jointName) const;

        IJoint * getJoint(const JointIndex index);

        const IJoint * getJoint(const JointIndex index) const;

        /**
         *
         * @return the JointIndex of the added joint, or JOINT_INVALID_INDEX if
         *         there was an error in adding the joint.
         */
        JointIndex addJoint(std::string & jointName, const IJoint * joint);
    };


}

#endif /* IDYNTREE_LINK_H */