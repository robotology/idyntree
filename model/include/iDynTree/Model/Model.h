/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_MODEL_H
#define IDYNTREE_MODEL_H

#include <iDynTree/Core/Link.h>

#include <iDynTree/Model/Indeces.h>


namespace iDynTree
{

    /**
     * Class that represents a generic multibody model.
     *
     *
     * \ingroup iDynTreeModel
     */
    class Model
    {
    private:
        std::vector<Link> links;
        std::vector<IJoint *> joints;

        std::vector<std::string> linkNames;
        std::vector<std::string> jointNames;

    public:
        /**
         * Costructor
         */
        Model();

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
        const std::string & getLinkName(const LinkIndex linkIndex) const;

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
        const std::string & getJointName(const JointIndex & index) const;

        /**
         * Get the index of a joint, given a jointName.
         * If the jointName is not found in the model,
         * return JOINT_INVALID_INDEX .
         *
         */
        JointIndex getJointIndex(const std::string & jointName) const;

        IJoint * getJoint();

        const IJoint * getJoint() const;

        /**
         *
         * @return the JointIndex of the added joint, or JOINT_INVALID_INDEX if
         *         there was an error in adding the joint. 
         */
        JointIndex addJoint(std::string & jointName, const IJoint * joint);
    };


}

#endif /* IDYNTREE_LINK_H */