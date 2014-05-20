/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#include <iCub/iDynTree/iCubTree.h>
#include <iCub/iDynTree/idyn2kdl_icub.h>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <iCub/skinDynLib/common.h>

#ifdef CODYCO_USES_URDFDOM
//Urdf import from kdl_format_io
#include <kdl_format_io/urdf_import.hpp>
#endif

using namespace iCub::skinDynLib;

namespace iCub {
namespace iDynTree {

const std::string ICUB_IMU_LINK_NAME = "imu_frame";

iCubTree::iCubTree(iCubTree_version_tag version, std::string imu_link_name, iCubTree_serialization_tag serial_tag, unsigned int verbose)
{
#ifdef CODYCO_USES_URDFDOM
if( !version.uses_urdf )
{
#endif
    yarp::sig::Vector q_min_yarp, q_max_yarp;
    KDL::JntArray q_min_kdl, q_max_kdl;
    //Allocate an old iCubWholeBody object, with right version
    iCub::iDyn::version_tag ver;
    ver.head_version = version.head_version;
    ver.legs_version = version.legs_version;
    bool ft_feet = version.feet_ft;

    //std::cout << "Creating iCubTree with head version " << ver.head_version << " and legs version " << ver.legs_version << std::endl;

    iCub::iDyn::iCubWholeBody icub_idyn(ver);

    //Convert it to a KDL::Tree (this preserve all the frame of reference, is the conversion to URDF that changes them)
    KDL::Tree icub_kdl;
    bool ret = toKDL(icub_idyn,icub_kdl,q_min_kdl,q_max_kdl,serial_tag,ft_feet);
    assert(ret);
    if( !ret ) {
        if( verbose ) { std::cerr << "iCubTree: error in costructor" << std::endl; }
        return;
    }

    //Construct F/T sensor name list
    std::vector< std::string > ft_names(0);
    std::vector<KDL::Frame> child_sensor_transforms(0);
    KDL::Frame kdlFrame;

    ft_names.push_back("l_arm_ft_sensor");
    ft_names.push_back("r_arm_ft_sensor");
    ft_names.push_back("l_leg_ft_sensor");
    if(ft_feet) {
        ft_names.push_back("l_foot_ft_sensor");
    }
    ft_names.push_back("r_leg_ft_sensor");
    if(ft_feet) {
        ft_names.push_back("r_foot_ft_sensor");
    }

    //Define an explicit serialization of the links and the DOFs of the iCub
    //The DOF serialization done in icub_kdl construction is ok
    KDL::CoDyCo::TreeSerialization serial = KDL::CoDyCo::TreeSerialization(icub_kdl);

    KDL::CoDyCo::TreePartition icub_partition = get_iCub_partition(serial,ft_feet);

    this->constructor(icub_kdl,ft_names,imu_link_name,serial,icub_partition);

    //Set joint limits

    int jjj;
    q_min_yarp.resize(q_min_kdl.rows());
    q_max_yarp.resize(q_max_kdl.rows());
    for(jjj=0; jjj<(int)q_min_kdl.rows(); jjj++) { q_min_yarp(jjj) = q_min_kdl(jjj); }
    for(jjj=0; jjj<(int)q_max_kdl.rows(); jjj++) { q_max_yarp(jjj) = q_max_kdl(jjj); }

    this->setJointBoundMin(q_min_yarp);
    this->setJointBoundMax(q_max_yarp);

    return;
#ifdef CODYCO_USES_URDFDOM
}
else
{
    std::string urdf_file = version.urdf_file;
    yarp::sig::Vector q_min_yarp, q_max_yarp;
    KDL::JntArray q_min_kdl, q_max_kdl;
    //Allocate an old iCubWholeBody object, with right version
    iCub::iDyn::version_tag ver;
    ver.head_version = version.head_version;
    ver.legs_version = version.legs_version;
    bool ft_feet = version.feet_ft;

    //std::cout << "Creating iCubTree with head version " << ver.head_version << " and legs version " << ver.legs_version << std::endl;

    iCub::iDyn::iCubWholeBody icub_idyn(ver);

    //Convert it to a KDL::Tree (this preserve all the frame of reference, is the conversion to URDF that changes them)
    KDL::Tree icub_kdl;

    bool ret = kdl_format_io::treeFromUrdfFile(urdf_file,icub_kdl);

    assert(ret);
    if( !ret ) {
        if( verbose ) { std::cerr << "iCubTree: error in costructor" << std::endl; }
        return;
    }

    //Construct F/T sensor name list
    std::vector< std::string > ft_names(0);
    std::vector<KDL::Frame> child_sensor_transforms(0);
    KDL::Frame kdlFrame;

    ft_names.push_back("l_arm_ft_sensor");
    ft_names.push_back("r_arm_ft_sensor");
    ft_names.push_back("l_leg_ft_sensor");
    if(ft_feet) {
        ft_names.push_back("l_foot_ft_sensor");
    }
    ft_names.push_back("r_leg_ft_sensor");
    if(ft_feet) {
        ft_names.push_back("r_foot_ft_sensor");
    }

    //Define an explicit serialization of the links and the DOFs of the iCub
    //The DOF serialization done in icub_kdl construction is ok
    KDL::CoDyCo::TreeSerialization serial = KDL::CoDyCo::TreeSerialization(icub_kdl);

    KDL::CoDyCo::TreePartition icub_partition = get_iCub_partition(serial,ft_feet);

    this->constructor(icub_kdl,ft_names,imu_link_name,serial,icub_partition);

    //Set joint limits

    int jjj;
    q_min_yarp.resize(q_min_kdl.rows());
    q_max_yarp.resize(q_max_kdl.rows());
    for(jjj=0; jjj<(int)q_min_kdl.rows(); jjj++) { q_min_yarp(jjj) = q_min_kdl(jjj); }
    for(jjj=0; jjj<(int)q_max_kdl.rows(); jjj++) { q_max_yarp(jjj) = q_max_kdl(jjj); }

    this->setJointBoundMin(q_min_yarp);
    this->setJointBoundMax(q_max_yarp);

    return;
}
#endif
}


#ifdef CODYCO_USES_URDFDOM
iCubTree::iCubTree(std::string urdf_file,
                   iCubTree_version_tag version,
                   std::string imu_link_name,
                   iCubTree_serialization_tag serial_tag,
                   unsigned int verbose)
{
    yarp::sig::Vector q_min_yarp, q_max_yarp;
    KDL::JntArray q_min_kdl, q_max_kdl;
    //Allocate an old iCubWholeBody object, with right version
    iCub::iDyn::version_tag ver;
    ver.head_version = version.head_version;
    ver.legs_version = version.legs_version;
    bool ft_feet = version.feet_ft;

    //std::cout << "Creating iCubTree with head version " << ver.head_version << " and legs version " << ver.legs_version << std::endl;

    iCub::iDyn::iCubWholeBody icub_idyn(ver);

    //Convert it to a KDL::Tree (this preserve all the frame of reference, is the conversion to URDF that changes them)
    KDL::Tree icub_kdl;

    bool ret = kdl_format_io::treeFromUrdfFile(urdf_file,icub_kdl);

    assert(ret);
    if( !ret ) {
        if( verbose ) { std::cerr << "iCubTree: error in costructor" << std::endl; }
        return;
    }

    //Construct F/T sensor name list
    std::vector< std::string > ft_names(0);
    std::vector<KDL::Frame> child_sensor_transforms(0);
    KDL::Frame kdlFrame;

    ft_names.push_back("l_arm_ft_sensor");
    ft_names.push_back("r_arm_ft_sensor");
    ft_names.push_back("l_leg_ft_sensor");
    if(ft_feet) {
        ft_names.push_back("l_foot_ft_sensor");
    }
    ft_names.push_back("r_leg_ft_sensor");
    if(ft_feet) {
        ft_names.push_back("r_foot_ft_sensor");
    }

    //Define an explicit serialization of the links and the DOFs of the iCub
    //The DOF serialization done in icub_kdl construction is ok
    KDL::CoDyCo::TreeSerialization serial = KDL::CoDyCo::TreeSerialization(icub_kdl);

    KDL::CoDyCo::TreePartition icub_partition = get_iCub_partition(serial,ft_feet);

    this->constructor(icub_kdl,ft_names,imu_link_name,serial,icub_partition);

    //Set joint limits

    int jjj;
    q_min_yarp.resize(q_min_kdl.rows());
    q_max_yarp.resize(q_max_kdl.rows());
    for(jjj=0; jjj<(int)q_min_kdl.rows(); jjj++) { q_min_yarp(jjj) = q_min_kdl(jjj); }
    for(jjj=0; jjj<(int)q_max_kdl.rows(); jjj++) { q_max_yarp(jjj) = q_max_kdl(jjj); }

    this->setJointBoundMin(q_min_yarp);
    this->setJointBoundMax(q_max_yarp);

    return;
}
#endif

KDL::CoDyCo::TreePartition iCubTree::get_iCub_partition(const KDL::CoDyCo::TreeSerialization & icub_serialization,bool ft_feet)
{
    //Define an explicit partition of the links and the DOFs of the iCub
    //The parts are defined in http://wiki.icub.org/wiki/ICub_joints
    //The parts ID are instead definde in skinDynLib http://wiki.icub.org/iCub_documentation/common_8h_source.html
    KDL::CoDyCo::TreePart head(HEAD,BodyPart_s[HEAD]);
    head.addDOF(icub_serialization.getDOFID("neck_pitch"));
    head.addDOF(icub_serialization.getDOFID("neck_roll"));
    head.addDOF(icub_serialization.getDOFID("neck_yaw"));

    head.addLink(icub_serialization.getLinkID("neck_1"));
    head.addLink(icub_serialization.getLinkID("neck_2"));
    head.addLink(icub_serialization.getLinkID("head"));
    head.addLink(icub_serialization.getLinkID(ICUB_IMU_LINK_NAME ));

    KDL::CoDyCo::TreePart torso(TORSO,BodyPart_s[TORSO]);
    torso.addDOF(icub_serialization.getDOFID("torso_pitch"));
    torso.addDOF(icub_serialization.getDOFID("torso_roll"));
    torso.addDOF(icub_serialization.getDOFID("torso_yaw"));

    torso.addLink(icub_serialization.getLinkID("root_link"));
    torso.addLink(icub_serialization.getLinkID("lap_belt_1"));
    torso.addLink(icub_serialization.getLinkID("chest"));
    torso.addLink(icub_serialization.getLinkID("lap_belt_2"));
    torso.addLink(icub_serialization.getLinkID("torso"));


    KDL::CoDyCo::TreePart left_arm(LEFT_ARM,BodyPart_s[LEFT_ARM]);
    left_arm.addDOF(icub_serialization.getDOFID("l_shoulder_pitch"));
    left_arm.addDOF(icub_serialization.getDOFID("l_shoulder_roll"));
    left_arm.addDOF(icub_serialization.getDOFID("l_shoulder_yaw"));
    left_arm.addDOF(icub_serialization.getDOFID("l_elbow"));
    left_arm.addDOF(icub_serialization.getDOFID("l_wrist_prosup"));
    left_arm.addDOF(icub_serialization.getDOFID("l_wrist_pitch"));
    left_arm.addDOF(icub_serialization.getDOFID("l_wrist_yaw"));

    //The link serialization is done in a way to be compatible with skinDynLib
    //(so the the upper part of the forerarm is shifted at the end)
    left_arm.addLink(icub_serialization.getLinkID("l_shoulder_1"));
    left_arm.addLink(icub_serialization.getLinkID("l_shoulder_2"));
    left_arm.addLink(icub_serialization.getLinkID("l_arm"));
    left_arm.addLink(icub_serialization.getLinkID("l_elbow_1"));
    left_arm.addLink(icub_serialization.getLinkID("l_forearm"));
    left_arm.addLink(icub_serialization.getLinkID("l_wrist_1"));
    left_arm.addLink(icub_serialization.getLinkID("l_hand"));
    //new links
    left_arm.addLink(icub_serialization.getLinkID("l_upper_arm"));
    left_arm.addLink(icub_serialization.getLinkID("l_gripper"));


    KDL::CoDyCo::TreePart right_arm(RIGHT_ARM,BodyPart_s[RIGHT_ARM]);
    right_arm.addDOF(icub_serialization.getDOFID("r_shoulder_pitch"));
    right_arm.addDOF(icub_serialization.getDOFID("r_shoulder_roll"));
    right_arm.addDOF(icub_serialization.getDOFID("r_shoulder_yaw"));
    right_arm.addDOF(icub_serialization.getDOFID("r_elbow"));
    right_arm.addDOF(icub_serialization.getDOFID("r_wrist_prosup"));
    right_arm.addDOF(icub_serialization.getDOFID("r_wrist_pitch"));
    right_arm.addDOF(icub_serialization.getDOFID("r_wrist_yaw"));

    //The link serialization is done in a way to be compatible with skinDynLib
    //(so the the upper part of the forerarm is shifted at the end)
    right_arm.addLink(icub_serialization.getLinkID("r_shoulder_1"));
    right_arm.addLink(icub_serialization.getLinkID("r_shoulder_2"));
    right_arm.addLink(icub_serialization.getLinkID("r_arm"));
    right_arm.addLink(icub_serialization.getLinkID("r_elbow_1"));
    right_arm.addLink(icub_serialization.getLinkID("r_forearm"));
    right_arm.addLink(icub_serialization.getLinkID("r_wrist_1"));
    right_arm.addLink(icub_serialization.getLinkID("r_hand"));
    //new links
    right_arm.addLink(icub_serialization.getLinkID("r_upper_arm"));
    right_arm.addLink(icub_serialization.getLinkID("r_gripper"));

    KDL::CoDyCo::TreePart left_leg(LEFT_LEG,BodyPart_s[LEFT_LEG]);
    left_leg.addDOF(icub_serialization.getDOFID("l_hip_pitch"));
    left_leg.addDOF(icub_serialization.getDOFID("l_hip_roll"));
    left_leg.addDOF(icub_serialization.getDOFID("l_hip_yaw"));
    left_leg.addDOF(icub_serialization.getDOFID("l_knee"));
    left_leg.addDOF(icub_serialization.getDOFID("l_ankle_pitch"));
    left_leg.addDOF(icub_serialization.getDOFID("l_ankle_roll"));

    //The link serialization is done in a way to be compatible with skinDynLib
    //(so the the upper part of the forerarm is shifted at the end)
    left_leg.addLink(icub_serialization.getLinkID("l_hip_1"));
    left_leg.addLink(icub_serialization.getLinkID("l_hip_2"));
    left_leg.addLink(icub_serialization.getLinkID("l_thigh"));
    left_leg.addLink(icub_serialization.getLinkID("l_shank"));
    left_leg.addLink(icub_serialization.getLinkID("l_ankle_1"));
    left_leg.addLink(icub_serialization.getLinkID("l_foot"));
    //new links
    left_leg.addLink(icub_serialization.getLinkID("l_hip_3"));
    if( ft_feet ) {  left_leg.addLink(icub_serialization.getLinkID("l_upper_foot")); }
    left_leg.addLink(icub_serialization.getLinkID("l_sole"));


    KDL::CoDyCo::TreePart right_leg(RIGHT_LEG,BodyPart_s[RIGHT_LEG]);
    right_leg.addDOF(icub_serialization.getDOFID("r_hip_pitch"));
    right_leg.addDOF(icub_serialization.getDOFID("r_hip_roll"));
    right_leg.addDOF(icub_serialization.getDOFID("r_hip_yaw"));
    right_leg.addDOF(icub_serialization.getDOFID("r_knee"));
    right_leg.addDOF(icub_serialization.getDOFID("r_ankle_pitch"));
    right_leg.addDOF(icub_serialization.getDOFID("r_ankle_roll"));

    //The link serialization is done in a way to be compatible with skinDynLib
    //(so the the upper part of the forerarm is shifted at the end)
    right_leg.addLink(icub_serialization.getLinkID("r_hip_1"));
    right_leg.addLink(icub_serialization.getLinkID("r_hip_2"));
    right_leg.addLink(icub_serialization.getLinkID("r_thigh"));
    right_leg.addLink(icub_serialization.getLinkID("r_shank"));
    right_leg.addLink(icub_serialization.getLinkID("r_ankle_1"));
    right_leg.addLink(icub_serialization.getLinkID("r_foot"));
    //new links
    right_leg.addLink(icub_serialization.getLinkID("r_hip_3"));
    if( ft_feet ) {  right_leg.addLink(icub_serialization.getLinkID("r_upper_foot")); }
    right_leg.addLink(icub_serialization.getLinkID("r_sole"));


    KDL::CoDyCo::TreePartition partition;
    partition.addPart(torso);
    partition.addPart(head);
    partition.addPart(left_arm);
    partition.addPart(right_arm);
    partition.addPart(left_leg);
    partition.addPart(right_leg);

    #ifndef NDEBUG
    //std::cout << "iCub partition " << partition.toString() << std::endl;
    #endif

    return partition;
}

iCubTree::~iCubTree() {}

}
}
