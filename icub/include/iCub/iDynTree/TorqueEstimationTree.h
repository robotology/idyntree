/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version)
 *
 */

#ifndef TORQUE_ESTIMATION_TREE_H
#define TORQUE_ESTIMATION_TREE_H

#include <kdl_format_io/urdf_sensor_import.hpp>

#include <iCub/iDynTree/DynTree.h>

#include <iCub/skinDynLib/dynContactList.h>

#include <iostream>

namespace iCub
{

namespace iDynTree
{
/**
 *  \ingroup iDynTree
 *
 * Class that instantiate a DynTree object configured for
 * Joint Torque estimation
 */
class TorqueEstimationTree : public DynTree
{
    private:
    //dynContact stuff
    std::vector< iCub::skinDynLib::dynContactList > contacts; /**< a vector of dynContactList, one for each dynamic subgraph */

     //DynTreeContact data structures
     std::vector<int> link2subgraph_index; /**< for each link, return the correspondent dynamics subgraph index */
     std::vector<bool> link_is_subgraph_root; /**< for each link, return if it is a subgraph root */
     std::vector<int> subgraph_index2root_link; /**< for each subgraph, return the index of the root */
     bool are_contact_estimated;

     int getSubGraphIndex(int link_index) {
         assert(link_index >= 0);
         assert(link_index < (int)link2subgraph_index.size());
         assert((int)link2subgraph_index.size() == this->getNrOfLinks());
         return link2subgraph_index[link_index];
     }

     bool isSubGraphRoot(int link_index) {
         assert((int)link_is_subgraph_root.size() == this->getNrOfLinks());
         return link_is_subgraph_root[link_index];
     }

     int buildSubGraphStructure(const std::vector<std::string> & ft_names);

     bool generateSensorsTree(const std::vector<std::string> & ft_names,
                                 const std::vector<bool> & is_measure_direction_child_to_parent);

     /**
      * Get the A e b local to a link, querying the contacts list and the FT sensor list
      *
      */
     yarp::sig::Vector getLinkLocalAb_contacts(int global_index, yarp::sig::Matrix & A, yarp::sig::Vector & b);

     bool isFTsensor(const std::string & joint_name, const std::vector<std::string> & ft_sensors) const;
     std::vector<yarp::sig::Matrix> A_contacts; /**< for each subgraph, the A regressor matrix of unknowns \todo use Eigen */
     std::vector<yarp::sig::Vector> b_contacts; /**< for each subgraph, the b vector of known terms \todo use Eigen */
     std::vector<yarp::sig::Vector> x_contacts; /**< for each subgraph, the x vector of unknowns */

     std::vector<KDL::Wrench> b_contacts_subtree; /**< for each link, the b vector of known terms of the subtree starting at that link expressed in the link frame*/

     /**
      * Preliminary version. If there are performance issues, this function
      * has several space for improvement.
      *
      */
     void buildAb_contacts();

     /** store contacts results */
     void store_contacts_results();


    public:

    /**
     * Constructor for TorqueEstimationTree
     *
     *
     * @param urdf_filename
     * @param verbose
     */
    TorqueEstimationTree(std::string urdf_filename,
                         std::vector<std::string> dof_serialization=std::vector<std::string>(0),
                         std::vector<std::string> ft_serialization=std::vector<std::string>(0),
                         std::string fixed_link="", unsigned int verbose=0);

    TorqueEstimationTree(KDL::Tree & icub_kdl,
                         std::vector<kdl_format_io::FTSensorData> ft_sensors,
                         std::vector<std::string> dof_serialization,
                         std::vector<std::string> ft_serialization,
                         yarp::sig::Vector & q_min, yarp::sig::Vector & q_max,
                         std::string fixed_link, unsigned int verbose);

    TorqueEstimationTree(KDL::Tree& icub_kdl,
                                          std::vector< kdl_format_io::FTSensorData > ft_sensors,
                                          std::vector< std::string > ft_serialization,
                                          std::string fixed_link, unsigned int verbose=0);

    void TorqueEstimationConstructor(KDL::Tree & icub_kdl,
                                std::vector<kdl_format_io::FTSensorData> ft_sensors,
                                std::vector<std::string> dof_serialization,
                                std::vector<std::string> ft_serialization,
                                yarp::sig::Vector & q_min, yarp::sig::Vector & q_max,
                                std::string fixed_link, unsigned int verbose);

    /**
     * This function enables interoperability between the iDynTree library
     * and the iCub skinDynLib library.
     *
     * This function can be used to assign a match between a skinDynLib link,
     * and a iDynTree link.
     * In skinDynLib a link is represented by two numbers:
     *   * the bodyPart, a numeric id representing a part of the robot
     *   * the linkIndex, a progressive numeric id uniquely identifyng  the link in the part
     * We associate this to two iDynTree concept
     *   * the link_name of the considered link
     *   * the frame_name of the frame used by skinDynLib when dealing with that link.
     *         This frame can be the link frame, in this case frame_name == link_name,
     *          otherwise another frame can be used, under the constraint that this frame
     *          must be rigidly attached to the considered link.
     *
     */
    bool addSkinDynLibAlias(const std::string iDynTree_link_name, const std::string iDynTree_frame_name,
                            const int skinDynLib_body_part, const int skinDynLib_link_index);

    /**
     * Retrieve the skinDynLib alias of a link, added to the class using the addSkinDynLibAlias method.
     */
    bool getSkinDynLibAlias(const std::string iDynTree_link_name, std::string & iDynTree_frame_name,
                            int & skinDynLib_body_part, int & skinDynLib_link_index) ;

    /**
     * Retrieve the skinDynLib alias of a link, added to the class using the addSkinDynLibAlias method.
     */
    bool getSkinDynLibAlias(const int iDynTree_link_index,
                             int & iDynTree_frame_index,
                             int & skinDynLib_body_part,
                             int & skinDynLib_link_index);

    /**
     * Convert a skinDynLib identifier to a iDynTree link/frame identifier.
     */
    bool skinDynLib2iDynTree(const int skinDynLib_body_part,
                             const int skinDynLib_link_index,
                             int & iDynTree_link_index,
                             int & iDynTree_frame_index);
    /**
     * Remove a alias in the form (body_part, link_index) for a link
     */
    bool removeSkinDynLibAlias(std::string link);

    /**
     * Estimate the external contacts, supplied by the setContacts call
     * for each dynamical subtree
     *
     */
    virtual bool estimateContactForcesFromSkin();

    //@}
    /** @name Methods related to contact forces
    *  This methods are related both to input and output of the esimation:
    *  the iCub::skinDynLib::dynContactList is used both to specify the
    *  unkown contacts via setContacts, and also to get the result of the
    *  estimation via getContacts
    *
    *  \note If for a given subtree no contact is given, a default contact
    *  is assumed, for example ad the end effector
    */
    //@{

    /**
    * Set the unknown contacts
    * @param contacts_list the list of the contacts on the DynTree
    * @return true if operation succeeded, false otherwise
    */
    virtual bool setContacts(const iCub::skinDynLib::dynContactList &contacts_list);

    /**
    * Get the contacts list, containing the results of the estimation if
    * estimateContacts was called
    * @return A reference to the external contact list
    */
    virtual const iCub::skinDynLib::dynContactList getContacts() const;

    /**
      * Get a list of wrenches that are the internal dynamics (base link M(q)ddq + C(q,dq)dq + n(q))
      * of each subtree, expressed in the world reference frame, with respect to the world origin
      */
    std::vector<yarp::sig::Vector> getSubTreeInternalDynamics();

    //@}


    virtual ~TorqueEstimationTree();
};

}//end namespace

}

#endif
