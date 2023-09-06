// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODEL_H
#define IDYNTREE_MODEL_H

#include <iDynTree/IJoint.h>
#include <iDynTree/Link.h>
#include <iDynTree/Transform.h>

#include <iDynTree/Indices.h>
#include <iDynTree/Sensors.h>
#include <iDynTree/SolidShapes.h>

#include <cstdlib>
#include <string>
#include <vector>

namespace iDynTree
{
    class Traversal;

    struct Neighbor
    {
        LinkIndex neighborLink;
        JointIndex neighborJoint;
    };

    /**
     * Class that represents a generic multibody model.
     *
     * A model is composed by rigid bodies (i.e. links) connected
     * by joints. Each joint can have from 0 to 6 degrees of freedom.
     *
     * Each link has a "link frame" rigidly attached to it.
     * Additionally, other rigidly attachable frames can be defined for each link.
     *
     * The model contains also a serialization for the different elements,
     * i.e. a function between:
     *  * joint names and the integers 0..getNrOfJoints()-1
     *  * dof   names and the integers 0..getNrOfDOFs()-1
     *  * link  names and the integers 0..getNrOfLinks()-1
     *  * frame names and the integers 0..getNrOfFrames()-1
     *
     * For simplicity, these mappings are build when building the model.
     * In particular the link and joint indices are assigned when the links
     * and joint are added to the model using the `addLink` and `addJoint`
     * methods.
     *
     * The DOF indices are also assigned when the joint is added to the model
     * with the addJoint method. For example if a model is composed only of
     * 0 or 1 DOF joints and the 1 DOFs joints are added before the 0 DOFs then
     * the joint index and dof index for 1 DOF joints will be coincident (this is
     * how the URDF parser is actually implemented). For this reason the current 
     * implementation does not have a concept of DOF explicit identifier, 
     * i.e. a getDOFName(DOFIndex dofIndex) method does not exist.
     *
     * The frame indices between 0 and getNrOfLinks()-1 are always assigned to the
     * "main" link frame of the link with the same index. The frame indices
     * between getNrOfLinks() and getNrOfFrames()-1 are assigned when the additional
     * frame is added to the model with the addAdditionalFrameToLink call. All the additional
     * frame indices are incremented by 1 whenever a new link is added, to ensure that
     * its "link frame" has a frame index in the 0...getNrOfLinks()-1 range.
     *
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

        /** Vector of additional frames.
         *  The element additionalFrames[frameOffset] will be the link_H_frame transform  of the frame with
         *  FrameIndex getNrOfLinks() + frameOffset .
         */
        std::vector<Transform> additionalFrames;

        /**
         *  Vector of link indices corresponding to an additional frame.
         *  The element additionalFrameNames[frameOffset] will be the link_H_frame transform  of the frame with
         *  FrameIndex getNrOfLinks() + frameOffset .
         */
        std::vector<LinkIndex> additionalFramesLinks;

        /** Vector of link names, matches the index of each link to its name. */
        std::vector<std::string> linkNames;

        /** Vector of joint names, matches the index of each joint to its name. */
        std::vector<std::string> jointNames;

        /**
         *  Vector of additional frame names.
         *  The element frameNames[frameOffset] will be the name of the frame with
         *  FrameIndex getNrOfLinks() + frameOffset .
         */
        std::vector<std::string> frameNames;

        /** Adjacency lists: match each link index to a list of its neighbors,
            and the joint connecting to them. */
        std::vector< std::vector<Neighbor> > neighbors;

        /** Vector containing the package directories associated to the model. */
        std::vector<std::string> packageDirs;

        /**
         * Most data structures are not undirected, so we store the original
         * root of the tree, to provide a default root for Traversal generation.
         */
        LinkIndex defaultBaseLink;

        /**
         * Cache number of position coordinaties of the model.
         * If all joints are 0 or 1 dofs, this is equal to nrOfDOFs.
         */
        unsigned int nrOfPosCoords;

        /**
         * Cached number of (internal) DOFs of the model.
         *  This is just the sum of all the getNrOfDOFs of the joint in the model.
         */
        unsigned int nrOfDOFs;

        /**
         * Solid shapes used for visualization.
         */
        ModelSolidShapes m_visualSolidShapes;

        /**
         * Solid shapes used for collision checking.
         */
        ModelSolidShapes m_collisionSolidShapes;

        /**
         * Sensors associated to the model.
         */
        SensorsList m_sensors;

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
         * Copy the model.
         *
         * Get a copy of the model.
         */
        Model copy() const;

        /**
         * Destructor
         *
         */
        virtual ~Model();

        /**
         * Get the number of links in the model.
         */
        size_t getNrOfLinks() const;

        /**
         * @return a vector containing all the directories of the meshes
         */
        const std::vector<std::string>& getPackageDirs() const;

        void setPackageDirs(const std::vector<std::string>& packageDirs);

        /**
         * Get the name of a link given its index, or
         * an LINK_INVALID_NAME string if linkIndex < 0 or >= getNrOfLinks()
         */
        std::string  getLinkName(const LinkIndex linkIndex) const;

        LinkIndex getLinkIndex(const std::string & linkName) const;

        /**
         * \brief Check if a given LinkIndex is valid.
         *
         * A link index is valid if is different from
         * LINK_INVALID_INDEX and 0 =< index < getNrOfLinks()-1
         *
         * @return true if the index is valid, false otherwise.
         */
        bool isValidLinkIndex(const LinkIndex index) const;

        LinkPtr getLink(const LinkIndex linkIndex);
        LinkConstPtr getLink(const LinkIndex linkIndex) const;

        LinkIndex addLink(const std::string & name, const Link & link);

        /**
         * Get number of joints in the model.
         */
        size_t getNrOfJoints() const;

        /**
         * Get the name of a joint given its index, or
         * an JOINT_INVALID_NAME if linkIndex < 0 or >= getNrOfLinks()
         */
        std::string getJointName(const JointIndex index) const;

        /**
         * Get the total mass of the robot
         */
        double getTotalMass() const;

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
         * \brief Check if a given JointIndex is valid.
         *
         * A joint index is valid if is different from
         * JOINT_INVALID_INDEX and 0 =< index < getNrOfJoints()-1
         *
         * @return true if the index is valid, false otherwise.
         */
        bool isValidJointIndex(const JointIndex index) const;

        /**
         * Check if a name is already used for a link in the model.
         *
         * @return true if a name is used by a link in a model, false otherwise.
         */
        bool isLinkNameUsed(const std::string linkName) const;

        /**
         * Check if a name is already used for a joint in the model.
         *
         * @return true if a name is used by a joint in a model, false otherwise.
         */
        bool isJointNameUsed(const std::string jointName) const;

        /**
         * Check if a name is already used for a frame in the model.
         *
         * \note this function will check the name of the links and the names of the additional frames.
         * @return true if a name is used by a frame in a model, false otherwise.
         */
        bool isFrameNameUsed(const std::string frameName) const;

        /**
         *
         * Add a joint to the model. The two links to which the joint is connected
         * are specified in the joint itself, throught the appropriate methods.
         * @return the JointIndex of the added joint, or JOINT_INVALID_INDEX if
         *         there was an error in adding the joint.
         */
        JointIndex addJoint(const std::string & jointName, IJointConstPtr joint);

        /**
         * Add a joint to the model, and specify the two links that are connected
         * by the specified joints. The setAttachedLinks of the passed joint
         * is called appropriately, to ensure consistency in the model.
         * @return the JointIndex of the added joint, or JOINT_INVALID_INDEX if
         *         there was an error in adding the joint.
         */
        JointIndex addJoint(const std::string & link1, const std::string & link2,
                            const std::string & jointName, IJointConstPtr joint);

        /**
         * Add a joint to the model, and add also a link.
         * The added joints connects an existing link of the model to the newly
         * added link.
         * The setAttachedLinks of the passed joint
         * is called appropriately, to ensure consistency in the model.
         * @return the JointIndex of the added joint, or JOINT_INVALID_INDEX if
         *         there was an error in adding the joint.
         */
        JointIndex addJointAndLink(const std::string & existingLink,
                                   const std::string & jointName, IJointConstPtr joint,
                                   const std::string & newLinkName, Link & newLink);

        /**
         * \brief Displace a link by inserting a new link and a new joint.
         * Displace a link by replacing it with a new link in the existing joint and insert new joint between the new link and the displaced link.
         * Inputs:
         * @param[in] existingJoint is where the new link will be inserted.
         * @param[in] unmovableLink is the link that was previously connected to the displaced link by the existingJoint.
         * @param[in] _unmovable_X_newLink is the transformation matrix from the unmovableLink to the new Link.
         * @param[in] jointName is the name of the new joint.
         * @param[in] joint is the new joint connecting the new link and the link that was displaced.
         * @param[in] newLinkName is the name of the new link.
         * @param[in] newLink is the new link that will be attached to the unmovable link using the existingJoint.
         * The setAttachedLinks of the new joint
         * and the existing joint are edited appropriately, to ensure consistency in the model.
         * @return the JointIndex of the added joint, or JOINT_INVALID_INDEX if
         *         there was an error in adding the joint.
         */
        JointIndex insertLinkToExistingJointAndAddJointForDisplacedLink(const std::string & existingJoint,
                                      const std::string & unmovableLink,
                                      const Transform& _unmovableLink_X_newLink,
                                   const std::string & jointName, IJointConstPtr joint,
                                   const std::string & newLinkName, Link & newLink);

        /**
         * Get the dimension of the vector used to parametrize the positions of the joints of the robot.
         * This number can be obtained by summing the getNrOfPosCoords of all the joints of the model.
         *
         * \warning This is *not* including the 6 degrees of freedom of the base.
         */
        size_t getNrOfPosCoords() const;

        /**
         * Get the number of degrees of freedom of the joint of the robot.
         * This number can be obtained by summing the getNrOfDOFs of all the joints of the model.
         *
         * \warning This is *not* including the 6 degrees of freedom of the base.
         */
        size_t getNrOfDOFs() const;

        /**
         * Get the number of frames in the model.
         *
         * \note this will return the sum of the number of link
         *       (as each link has an attached frame) and the total number
         *       of additional frames.
         *
         * @return the number of frames in the model.
         */
        size_t getNrOfFrames() const;

        /**
         * Add an additional frame to a link.
         *
         * \note This function has signature different from
         *       addJoint/addLink because the FrameIndex of the
         *       additional frame are invalidated at each call to
         *       the addLink. So we don't return the FrameIndex in this
         *       function, as the model construction should be completed
         *       before that FrameIndex are stored.
         *
         * @param[in] linkName the link to which attach the additional frame.
         * @param[in] frameName the name of the frame added to the model.
         * @param[in] link_H_frame the pose of added frame with respect to the link main frame,
         *                         expressed with a transform with:
         *                              refFrame: the main link frame
         *                              frame: the added frame
         * @return true if all went well, false if an error occured.
         *
         */
        bool addAdditionalFrameToLink(const std::string & linkName,
                                      const std::string & frameName,
                                      iDynTree::Transform link_H_frame);

        /**
         * Get the name of a frame given its index.
         *
         * @param[in] frameIndex the index of the frame whose name is requested.
         * @return the name of a frame given its index, or
         *         a FRAME_INVALID_NAME string if frameIndex < 0 or >= getNrOfFrames()
         */
        std::string  getFrameName(const FrameIndex frameIndex) const;

        /**
         * Get the index of a frame given its name.
         *
         * @param[in] frameName the name of the frame for which the index is requested.
         * @return    the index of the frame, of FRAME_INVALID_INDEX if frameName is not
         *            not a frame name.
         */
        FrameIndex getFrameIndex(const std::string & frameName) const;

        /**
         * \brief Check if a given FrameIndex is valid.
         *
         * A frame index is valid if is different from
         * FRAME_INVALID_INDEX and 0 =< index < getNrOfFrames()-1
         *
         * @return true if the index is valid, false otherwise.
         */
        bool isValidFrameIndex(const FrameIndex index) const;

        /**
         * Get the tranform of the frame with respect to the main
         * frame of the link, returned as link_H_frame tranform
         * with refFrame : the link main frame and frame : the
         *  frame with index frameIndex .
         *
         * @param[in] frameIndex the index of the frame for which transform is requested.
         * @return the link_H_frame transform, or an identity tranform
         *         if frameIndex < 0 or frameIndex >= getNrOfFrames .
         */
        Transform getFrameTransform(const FrameIndex frameIndex) const;

        /**
         * Get the link at which the frame with index frameIndex
         * is attached.
         *
         */
        LinkIndex getFrameLink(const FrameIndex frameIndex) const;

        /**
         * Get the additional frames of a specified link.
         *
         * @note The vector of returned frame index is ordered according to the frame index.
         * @warning This method searches linearly over all the frames.
         *
         * @param[in] link a LinkIndex of the specified link,
         * @param[out] frames a vector of FrameIndex of the frame indeces attached to the specified link index,
         * @return true if the specified link is a valid link, false otherwise.
         */
        bool getLinkAdditionalFrames(const LinkIndex lnkIndex, std::vector<FrameIndex>& frameIndeces) const;

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
         *
         * If no link are present in model, returns LINK_INVALID_INDEX.
         * If setDefaultBaseLink was never called but at least a link has been added
         * to the model, returns 0 (i.e. the index of the first link added to the model.
         * 
         * @return index of the default base link, if valid
         */
        LinkIndex getDefaultBaseLink() const;

        /**
         * Compute a Traversal of all the links in the Model, doing a Depth First Search starting
         * at the default base.
         *
         * \warning The traversal computed with this function contains pointers to the joints
         *          and links present in this model. Whenever these pointers are invalidated,
         *          for example because a link or a joint is added or the model is copied,
         *          the traversal need to be recomputed.
         *
         * \warning this function works only on Models without cycles.
         * @param[out] traversal traversal of all links in the model
         * @return true if all went well, false otherwise.
         */
        bool computeFullTreeTraversal(Traversal & traversal) const;

       /**
         * Compute a Traversal of all the links in the Model, doing a Depth First Search starting
         * at the given traversalBase.
         *
         * \warning The traversal computed with this function contains pointers to the joints
         *          and links present in this model. Whenever these pointers are invalidated,
         *          for example because a link or a joint is added or the model is copied,
         *          the traversal need to be recomputed.
         *.
         * \warning this function works only on Models without cycles.
         * @param[out] traversal traversal of all links in the model
         * @param[in]  traversalBase base (root) link in the traversal
         * @return true if all went well, false otherwise
         */
        bool computeFullTreeTraversal(Traversal & traversal, const LinkIndex traversalBase) const;

        /**
         * Get the inertial parameters of the links of the model in vector forms.
         *
         * This methods gets the inertial parameters (mass, center
         * of mass, 3D inertia matrix) of the links of the robot.
         *
         * The output vector of inertial parameters must have 10*getNrOfLinks() elements,
         * each 10 elements subvector corresponds to the inertial parameters of one link,
         * following the serialization induced by the link indices
         * (link 0 corresponds to elements 0-9, link 1 to 10-19, etc).
         *
         * The mapping between the SpatialInertia class and the Vector10 elements is the one
         * defined in SpatialInertia::asVector() method.
         *
         * @param[out] modelInertialParams vector of inertial parameters
         * @return true if all went well, false otherwise.
         *
         */
        bool getInertialParameters(VectorDynSize & modelInertialParams) const;

        /**
         * Update the inertial parameters of the links of the model.
         *
         * This methods modifies the inertial parameters (mass, center
         * of mass, 3D inertia matrix) of the links of the robot.
         *
         * The input vector of inertial parameters must have 10*getNrOfLinks() elements,
         * each 10 elements subvector corresponds to the inertial parameters of one link,
         * following the serialization induced by the link indices
         * (link 0 corresponds to elements 0-9, link 1 to 10-19, etc).
         *
         * The mapping between the SpatialInertia class and the Vector10 elements is the one
         * defined in SpatialInertia::asVector() method.
         *
         * @note For efficency reason, inertial parameters are not checked for full physical
         *       consistency before being update.
         *
         * @param[in] modelInertialParams vector of inertial parameters
         * @return true if all went well, false otherwise.
         *
         */
        bool updateInertialParameters(const VectorDynSize & modelInertialParams);

        /**
         * Get the ModelSolidShapes meant for visualization.
         *
         * @return a reference to ModelSolidShapes meant for visualization.
         */
        ModelSolidShapes& visualSolidShapes();

        /**
         * Get the ModelSolidShapes meant for visualization (const version)
         *
         * @return a reference to ModelSolidShapes meant for visualization.
         */
        const ModelSolidShapes& visualSolidShapes() const;

        /**
         * Get the ModelSolidShapes meant for collision checking.
         *
         * @return a reference to ModelSolidShapes meant for visualization.
         */
        ModelSolidShapes& collisionSolidShapes();


        /**
         * Get the ModelSolidShapes meant for collision checking (const version)
         *
         * @return a reference to ModelSolidShapes meant for visualization.
         */
        const ModelSolidShapes& collisionSolidShapes() const;

        /**
         * Get the (mutable) sensors associated with the model.
         *
         * @return a (mutable) reference to SensorsList associated with the model.
         */
        SensorsList& sensors();

        /**
         * Get the (const) sensors associated with the model.
         *
         * @return a (const) reference to SensorsList associated with the model.
         */
        const SensorsList& sensors() const;

        /**
         * \brief Get a printable representation of the Model.
         *
         * Useful for debugging.
         */
        std::string toString() const;

        /**
         * \brief Check if the model is valid.
         *
         * Useful for debugging.
         */
        bool isValid() const;

    };


}

#endif /* IDYNTREE_LINK_H */
