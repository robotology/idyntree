/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */


#include <iCub/iDynTree/idyn2kdl_icub.h>
#include <iCub/iKin/iKinFwd.h>
#include <yarp/math/Math.h>
#include <yarp/os/Log.h>

#include <kdl_codyco/treefksolverpos_iterative.hpp>

#include <kdl/frames_io.hpp>
#include <iCub/iDynTree/yarp_kdl.h>

using yarp::math::cat;

template<typename T, size_t N>
T * end(T (&ra)[N]) {
    return ra + N;
}

//specifing directly names (different) for joints and links
/*
bool names2links_joints(const std::vector<std::string> names,std::vector<std::string> & names_links,std::vector<std::string> & names_joints)
{
    names_links = names;
    names_joints = names;
    for(int i=0;i<(int)names.size();i++) {
        names_links[i] = names_links[i]+"_link";
        names_joints[i] = names_joints[i]+"_joint";
    }
    return true;
}
*/

// For more informations on this values, check
// https://github.com/robotology/codyco-modules/issues/49
const double root_link_weight = 4.72;
//const double head_weight      = 3.0;

KDL::RotationalInertia operator-(const KDL::RotationalInertia& Ia, const KDL::RotationalInertia& Ib){
    KDL::RotationalInertia result;
    Eigen::Map<Eigen::Matrix3d>(result.data)=Eigen::Map<const Eigen::Matrix3d>(Ia.data)-Eigen::Map<const Eigen::Matrix3d>(Ib.data);
    return result;
}

KDL::RigidBodyInertia operator-(const KDL::RigidBodyInertia& Ia, const KDL::RigidBodyInertia& Ib){
    double new_mass = Ia.getMass()-Ib.getMass();
    KDL::Vector new_cog = Ia.getCOG()-Ib.getCOG();

    KDL::RotationalInertia new_inertia_ref_frame = Ia.getRotationalInertia()-Ib.getRotationalInertia();

    KDL::RotationalInertia new_inertia_cog;

    Eigen::Vector3d c_eig=Eigen::Map<const Eigen::Vector3d>(new_inertia_cog.data);
    Eigen::Map<Eigen::Matrix3d>(new_inertia_cog.data)=Eigen::Map<const Eigen::Matrix3d>(new_inertia_ref_frame.data)+new_mass*(c_eig*c_eig.transpose()-c_eig.dot(c_eig)*Eigen::Matrix3d::Identity());

    return KDL::RigidBodyInertia(new_mass,new_cog,new_inertia_cog);
}

void addFrame(KDL::Tree & icub_kdl, KDL::Frame link_H_frame, std::string link_name, std::string frame_name)
{
    KDL::Segment kdlSegment = KDL::Segment(frame_name,KDL::Joint(frame_name+"_fixed_joint",KDL::Joint::None),link_H_frame);
    icub_kdl.addSegment(kdlSegment,link_name);
}

bool toKDL(const iCub::iDyn::iCubWholeBody & icub_idyn,
           const iCub::iDyn::version_tag   & version_idyn,
           KDL::Tree & icub_kdl,
           KDL::JntArray & q_min,
           KDL::JntArray & q_max,
           iCub::iDynTree::iCubTree_serialization_tag serial,
           bool ft_foot,
           bool add_root_weight,
           bool /*debug*/,
           bool ft_foot_iCubParis02,
           bool ft_foot_icubGazeboSim
          )
{
    std::cerr << "toKDL(..) function called with ft_foot_iCubParis02 " << ft_foot_iCubParis02
              << " and ft_foot_icubGazeboSim " << ft_foot_icubGazeboSim << std::endl;

    bool status_ok = true;
    //Joint names extracted from http://eris.liralab.it/wiki/ICub_joints

    //Default "fake" base link for humanoids URDF
    const std::string fake_root_name = "base_link";

    //Name of the base for the torso and the legs
    const std::string real_root_name = "root_link";

    //name of the base for the head and the arms
    #define ARMS_HEAD_BASE_NAME "chest"
    const std::string arms_head_base_name = ARMS_HEAD_BASE_NAME;

    icub_kdl = KDL::Tree(fake_root_name);

    std::vector<std::string> joints,links;


    //Creating torso
    KDL::Chain torso, old_torso;
    const char *torso_joints_cstr[] = {"torso_pitch","torso_roll","torso_yaw"};
    std::vector<std::string> torso_joints(torso_joints_cstr,end(torso_joints_cstr));
    const char *torso_links_cstr[] = {"torso_1","torso_2",ARMS_HEAD_BASE_NAME};
    std::vector<std::string> torso_links(torso_links_cstr,end(torso_links_cstr));
    //names2links_joints(torso_joints,links,joints);
    status_ok = idynChain2kdlChain(*(icub_idyn.lowerTorso->up),old_torso,torso_links,torso_joints);
    if(!status_ok) return false;

    //Creating head
    KDL::Chain head, old_head;
    const char *head_joints_cstr[] = {"neck_pitch","neck_roll","neck_yaw","imu_frame_fixed_joint"};
    std::vector<std::string> head_joints(head_joints_cstr,end(head_joints_cstr));
    //names2links_joints(head_joints,links,joints);
    const char *head_links_cstr[] = {"neck_1","neck_2","head","imu_frame"};
    std::vector<std::string> head_links(head_links_cstr,end(head_links_cstr));
    status_ok = idynChain2kdlChain(*(icub_idyn.upperTorso->up),old_head,head_links,head_joints);
    if(!status_ok) return false;


    //Creating left arm
    KDL::Chain la, old_la;
    const char *la_joints_cstr[] = {"l_shoulder_pitch", "l_shoulder_roll","l_shoulder_yaw" ,"l_arm_ft_sensor" , "l_elbow", "l_wrist_prosup", "l_wrist_pitch","l_wrist_yaw",};
    std::vector<std::string> la_joints(la_joints_cstr,end(la_joints_cstr));
    //names2links_joints(la_joints,links,joints);
    const char *la_links_cstr[] = {"l_shoulder_1", "l_shoulder_2","l_shoulder_3" ,"l_upper_arm" , "l_elbow_1", "l_forearm", "l_wrist_1","l_hand",};
    std::vector<std::string> la_links(la_links_cstr,end(la_links_cstr));
    KDL::Frame ft_la_H_sensor_dh_child;
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.upperTorso->left),*(icub_idyn.upperTorso->leftSensor),old_la,ft_la_H_sensor_dh_child,la_links,la_joints,"l_gripper");
    if(!status_ok) return false;


    //Creating right arm
    KDL::Chain ra, old_ra;
    const char *ra_joints_cstr[] = {"r_shoulder_pitch", "r_shoulder_roll","r_shoulder_yaw", "r_arm_ft_sensor", "r_elbow", "r_wrist_prosup", "r_wrist_pitch","r_wrist_yaw",};
    std::vector<std::string> ra_joints(ra_joints_cstr,end(ra_joints_cstr));
    //names2links_joints(ra_joints,links,joints);
    const char *ra_links_cstr[] = {"r_shoulder_1", "r_shoulder_2","r_shoulder_3" ,"r_upper_arm" , "r_elbow_1", "r_forearm", "r_wrist_1","r_hand",};
    std::vector<std::string> ra_links(ra_links_cstr,end(ra_links_cstr));
    KDL::Frame ft_ra_H_sensor_dh_child;
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.upperTorso->right),*(icub_idyn.upperTorso->rightSensor),old_ra,ft_ra_H_sensor_dh_child,ra_links,ra_joints,"r_gripper");
    if(!status_ok) return false;

    //Creating left leg
    KDL::Chain ll, old_ll;
    const char *ll_joints_cstr[] = {"l_hip_pitch", "l_hip_roll", "l_leg_ft_sensor", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll"};
    std::vector<std::string> ll_joints(ll_joints_cstr,end(ll_joints_cstr));
    //names2links_joints(ll_joints,links,joints);
    const char *ll_links_cstr[] = {"l_hip_1", "l_hip_2", "l_hip_3", "l_upper_leg", "l_lower_leg", "l_ankle_1", "l_foot"};
    std::vector<std::string> ll_links(ll_links_cstr,end(ll_links_cstr));
    KDL::Frame ft_ll_H_sensor_dh_child;
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.lowerTorso->left),
                                         *(icub_idyn.lowerTorso->leftSensor),old_ll,ft_ll_H_sensor_dh_child,ll_links,ll_joints,"l_sole");
    if(!status_ok) return false;


    //Creating right leg
    KDL::Chain rl, old_rl;
    const char *rl_joints_cstr[] = {"r_hip_pitch", "r_hip_roll", "r_leg_ft_sensor", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};
    std::vector<std::string> rl_joints(rl_joints_cstr,end(rl_joints_cstr));
    //names2links_joints(rl_joints,links,joints);
    const char *rl_links_cstr[] = {"r_hip_1", "r_hip_2", "r_hip_3", "r_upper_leg", "r_lower_leg", "r_ankle_1", "r_foot"};
    std::vector<std::string> rl_links(rl_links_cstr,end(rl_links_cstr));
    KDL::Frame ft_rl_H_sensor_dh_child;
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.lowerTorso->right),*(icub_idyn.lowerTorso->rightSensor),old_rl,ft_rl_H_sensor_dh_child,rl_links,rl_joints,"r_sole");
    if(!status_ok) return false;


    //Now that all the chain are created, it is possible to compose them
    //to create the iCub KDL::Tree

    //First we have to had the root_link, ( for now without RigidBodyInertia!)
    KDL::RigidBodyInertia I_root;
    if( !add_root_weight ) {
        I_root = KDL::RigidBodyInertia();
    } else {
        I_root = KDL::RigidBodyInertia(root_link_weight);
    }

        status_ok = icub_kdl.addSegment(KDL::Segment(real_root_name,KDL::Joint("base_fixed_joint",KDL::Joint::None),KDL::Frame::Identity(),I_root),fake_root_name);

    if(!status_ok) return false;


    //Adding the chains, selecting the default serialization
    KDL::Frame kdlFrame;
    idynMatrix2kdlFrame(icub_idyn.lowerTorso->HLeft,kdlFrame);
    addBaseTransformation(old_ll,ll,kdlFrame);

    idynMatrix2kdlFrame(icub_idyn.lowerTorso->HRight,kdlFrame);
    addBaseTransformation(old_rl,rl,kdlFrame);

    idynMatrix2kdlFrame(icub_idyn.lowerTorso->HUp,kdlFrame);
    addBaseTransformation(old_torso,torso,kdlFrame);

    //not using RBT because in the iCub it is an identity, and it is not clear is
    //semantical meaning (if is H_upper_lower or H_lower_upper )
    idynMatrix2kdlFrame(icub_idyn.upperTorso->HLeft,kdlFrame);
    addBaseTransformation(old_la,la,kdlFrame);

    idynMatrix2kdlFrame(icub_idyn.upperTorso->HRight,kdlFrame);
    addBaseTransformation(old_ra,ra,kdlFrame);

    idynMatrix2kdlFrame(icub_idyn.upperTorso->HUp,kdlFrame);
    addBaseTransformation(old_head,head,kdlFrame);

    //For the foot, it is possible that is present the ft sensor
    KDL::Frame ft_lf_H_sensor_child;
    KDL::Frame ft_rf_H_sensor_child;
    if( ft_foot )
    {
        // We need this option for two
        //If ft foot are present in legs
        //then the feet is v2
        iCub::iDyn::version_tag tag_feetV2;

        tag_feetV2.head_version = 2;
        tag_feetV2.legs_version = 2;

        iCub::iDyn::iCubWholeBody icub_idyn_feetv2(tag_feetV2);

        KDL::Chain llV2, old_llV2;
        status_ok = idynSensorChain2kdlChain(*(icub_idyn_feetv2.lowerTorso->left),*(icub_idyn_feetv2.lowerTorso->leftSensor),old_llV2,ft_ll_H_sensor_dh_child,ll_links,ll_joints,"l_sole");
        if(!status_ok) return false;

        //Creating right leg
        KDL::Chain rlV2, old_rlV2;
        status_ok = idynSensorChain2kdlChain(*(icub_idyn_feetv2.lowerTorso->right),*(icub_idyn_feetv2.lowerTorso->rightSensor),old_rlV2,ft_rl_H_sensor_dh_child,rl_links,rl_joints,"r_sole");
        if(!status_ok) return false;

        idynMatrix2kdlFrame(icub_idyn_feetv2.lowerTorso->HLeft,kdlFrame);
        addBaseTransformation(old_llV2,llV2,kdlFrame);

        idynMatrix2kdlFrame(icub_idyn_feetv2.lowerTorso->HRight,kdlFrame);
        addBaseTransformation(old_rlV2,rlV2,kdlFrame);
        //End legs v2 part


        //std::cerr << "toKDL: adding feet FT sensors" << std::endl;
        KDL::Chain no_ft_rl, no_ft_ll, no_ft_rlV2, no_ft_llV2;

        // We have three possible cases for the translation between the DH end effector frame
        // and the ft sensor frame:
        KDL::Frame ss_T_old_ee;
        KDL::Frame new_ee_T_old_ee;
        if( version_idyn.legs_version == 2 )
        {
            // If we have a iCub v2.5 leg kinematics, the iDyn ee frame origin is the projection of the
            // ft sensor origin, so there is just a slight offset of 0.004 meters
            ss_T_old_ee = KDL::Frame(KDL::Rotation(0,0,1,
                                               0,1,0,
                                               -1,0,0),
                                     KDL::Vector(0,0,0.004)); //transformation between the end effector and the sensor

            new_ee_T_old_ee = KDL::Frame(KDL::Rotation(1,0,0,
                                                       0,1,0,
                                                       0,0,1),
                                           KDL::Vector(0,0,0));
        } else
        {
            assert( version_idyn.legs_version == 1 );
            if( ft_foot_iCubParis02 )
            {
                // if we have the iCubParis02, i.e. a iCub with v2 legs and
                // a ft sensor moved towards the front of the foot, we will use the
                // v1 legs kinematics and a specific rototranslation between the
                // dh end effector using the v1 frame and the new ft sensor frame
                ss_T_old_ee = KDL::Frame(KDL::Rotation(0,0,1,
                                                       0,1,0,
                                                       -1,0,0),
                                             KDL::Vector(-0.008,0,-0.0143));

                new_ee_T_old_ee = KDL::Frame(KDL::Rotation(1,0,0,
                                                           0,1,0,
                                                           0,0,1),
                                                       KDL::Vector(0,0,0.0183));
            }
            else if (ft_foot_icubGazeboSim )
            {
                // if we have a 'fake' ft sensor with a v1 feet we have specific parmaters
                ss_T_old_ee = KDL::Frame(KDL::Rotation(0,0,1,
                                                       0,1,0,
                                                       -1,0,0),
                                             KDL::Vector(0.0,0,0.004));
                new_ee_T_old_ee = KDL::Frame::Identity();
            }
            else
            {
                // if we have a iCub with v1 legs but with ft sensors in the ankle
                // we have a specific rototranslation between the
                // dh end effector using the v1 frame and the new ft sensor frame
                ss_T_old_ee = KDL::Frame(KDL::Rotation(0,0,1,
                                                       0,1,0,
                                                       -1,0,0),
                                         KDL::Vector(-0.017,0,-0.001));

                new_ee_T_old_ee = KDL::Frame(KDL::Rotation(1,0,0,
                                                           0,1,0,
                                                           0,0,1),
                                                       KDL::Vector(0,0,0.0183));
            }
        }

        KDL::Frame old_ee_T_ss = ss_T_old_ee.Inverse();
        KDL::Frame old_ee_T_new_ee = new_ee_T_old_ee.Inverse();

        no_ft_rlV2 = rlV2;
        no_ft_llV2 = llV2;
        no_ft_rl = rl;
        no_ft_ll = ll;
        rl = KDL::Chain();
        ll = KDL::Chain();
        //add for each leg til ankle_1
        for(int iii=0; iii <= 5; iii++ ) { rl.addSegment(no_ft_rl.getSegment(iii)); ll.addSegment(no_ft_ll.getSegment(iii)); }
        KDL::Segment r_foot_legsV25_no_ft = no_ft_rlV2.getSegment(6);
        KDL::Segment l_foot_legsV25_no_ft = no_ft_llV2.getSegment(6);
        KDL::Segment r_foot_original_no_ft = no_ft_rl.getSegment(6);
        KDL::Segment l_foot_original_no_ft = no_ft_ll.getSegment(6);

        //Build new segments
        // mass extracted from the (hopefully correct) iCubHeidelberg01 )
        double ankle_2_weight = 0.2675;
        KDL::RigidBodyInertia r_upper_foot_I = KDL::RigidBodyInertia(ankle_2_weight);
        KDL::RigidBodyInertia l_upper_foot_I = KDL::RigidBodyInertia(ankle_2_weight);

        // use always feetV2 dynamics, but depending on the type of leg use feetV1 or feetV2 kinematics
        // see https://github.com/robotology-playground/icub-model-generator/issues/9 for more info
        KDL::Segment r_ankle_2("r_ankle_2",r_foot_original_no_ft.getJoint(),r_foot_original_no_ft.getFrameToTip()*old_ee_T_ss,r_upper_foot_I);
        KDL::Segment l_ankle_2("l_ankle_2",l_foot_original_no_ft.getJoint(),l_foot_original_no_ft.getFrameToTip()*old_ee_T_ss,l_upper_foot_I);

        KDL::RigidBodyInertia r_foot_new_I = ss_T_old_ee*r_foot_legsV25_no_ft.getInertia()-r_upper_foot_I;
        KDL::RigidBodyInertia l_foot_new_I = ss_T_old_ee*l_foot_legsV25_no_ft.getInertia()-l_upper_foot_I;
        KDL::Segment r_foot_new("r_foot",KDL::Joint("r_foot_ft_sensor"),KDL::Frame::Identity(),r_foot_new_I);
        KDL::Segment l_foot_new("l_foot",KDL::Joint("l_foot_ft_sensor"),KDL::Frame::Identity(),l_foot_new_I);

        ft_lf_H_sensor_child = KDL::Frame::Identity();
        ft_rf_H_sensor_child = KDL::Frame::Identity();

        KDL::Segment l_foot_dh_frame("l_foot_dh_frame",KDL::Joint("l_foot_dh_frame_fixed_joint"),ss_T_old_ee*old_ee_T_new_ee);
        KDL::Segment r_foot_dh_frame("r_foot_dh_frame",KDL::Joint("r_foot_dh_frame_fixed_joint"),ss_T_old_ee*old_ee_T_new_ee);

        rl.addSegment(r_ankle_2); rl.addSegment(r_foot_new); rl.addSegment(r_foot_dh_frame);

        ll.addSegment(l_ankle_2); ll.addSegment(l_foot_new); ll.addSegment(l_foot_dh_frame);

        if( ft_foot_icubGazeboSim )
        {
            std::cerr << "[DEBUG] Original  r_foot_original_no_ft.getFrameToTip(): " << r_foot_original_no_ft.getFrameToTip()  << std::endl;
            std::cerr << "[DEBUG] r_foot_original_no_ft.getFrameToTip()*old_ee_T_ss  " << r_foot_original_no_ft.getFrameToTip()*old_ee_T_ss << std::endl;
            std::cerr << "[DEBUG] r_ankle_2.getFrameToTip() " << r_ankle_2.getFrameToTip() << std::endl;
            std::cerr << "[DEBUG] r_foot_new.getFrameToTip() " << r_foot_new.getFrameToTip() << std::endl;
            std::cerr << "[DEBUG] ss_T_old_ee*old_ee_T_new_ee " << ss_T_old_ee*old_ee_T_new_ee << std::endl;
            std::cerr << "[DEBUG] r_foot_dh_frame.getFrameToTip() " << r_foot_dh_frame.getFrameToTip() << std::endl;

        }
    }

    //Get joint limits
    yarp::sig::Vector torso_min = icub_idyn.lowerTorso->up->getJointBoundMin();
    yarp::sig::Vector torso_max = icub_idyn.lowerTorso->up->getJointBoundMax();

    yarp::sig::Vector head_min = icub_idyn.upperTorso->up->getJointBoundMin();
    yarp::sig::Vector head_max = icub_idyn.upperTorso->up->getJointBoundMax();

    yarp::sig::Vector la_min = icub_idyn.upperTorso->left->getJointBoundMin();
    yarp::sig::Vector la_max = icub_idyn.upperTorso->left->getJointBoundMax();

    yarp::sig::Vector ra_min = icub_idyn.upperTorso->right->getJointBoundMin();
    yarp::sig::Vector ra_max = icub_idyn.upperTorso->right->getJointBoundMax();

    yarp::sig::Vector ll_min = icub_idyn.lowerTorso->left->getJointBoundMin();
    yarp::sig::Vector ll_max = icub_idyn.lowerTorso->left->getJointBoundMax();

    yarp::sig::Vector rl_min = icub_idyn.lowerTorso->right->getJointBoundMin();
    yarp::sig::Vector rl_max = icub_idyn.lowerTorso->right->getJointBoundMax();

    yarp::sig::Vector q_min_yarp;
    yarp::sig::Vector q_max_yarp;

    if( serial == iCub::iDynTree::IDYN_SERIALIZATION  ) {
        //Using serialization in iCubWholeBody
        icub_kdl.addChain(ll,real_root_name);
        icub_kdl.addChain(rl,real_root_name);
        icub_kdl.addChain(torso,real_root_name);
        icub_kdl.addChain(la,arms_head_base_name);
        icub_kdl.addChain(ra,arms_head_base_name);
        icub_kdl.addChain(head,arms_head_base_name);

        q_min_yarp = cat(ll_min,cat(rl_min,cat(torso_min,cat(la_min,cat(ra_min,head_min)))));
        q_max_yarp = cat(ll_max,cat(rl_max,cat(torso_max,cat(la_max,cat(ra_max,head_max)))));

    } else {
        assert(serial == iCub::iDynTree::SKINDYNLIB_SERIALIZATION);
        //Using serialization used in wholeBodyInterfaceYarp ans skinDynLib
        icub_kdl.addChain(torso,real_root_name);
        icub_kdl.addChain(head,arms_head_base_name);
        icub_kdl.addChain(la,arms_head_base_name);
        icub_kdl.addChain(ra,arms_head_base_name);
        icub_kdl.addChain(ll,real_root_name);
        icub_kdl.addChain(rl,real_root_name);

        q_min_yarp = cat(torso_min,cat(head_min,cat(la_min,cat(ra_min,cat(ll_min,rl_min)))));
        q_max_yarp = cat(torso_max,cat(head_max,cat(la_max,cat(ra_max,cat(ll_max,rl_max)))));

    }

    q_min.resize(q_min_yarp.size());
    q_max.resize(q_max_yarp.size());

    for(unsigned jjj=0; jjj<q_min.rows(); jjj++) { q_min(jjj) = q_min_yarp(jjj); }
    for(unsigned jjj=0; jjj<q_max.rows(); jjj++) { q_max(jjj) = q_max_yarp(jjj); }

    //REP 120

    KDL::Segment kdlSegment = KDL::Segment("torso",KDL::Joint("torso_joint",KDL::Joint::None));
    icub_kdl.addSegment(kdlSegment,arms_head_base_name);

    // Export FT Sensor frames

    if( !ft_foot )
    {
        addFrame(icub_kdl,KDL::Frame::Identity(),"r_foot","r_foot_dh_frame");
        addFrame(icub_kdl,KDL::Frame::Identity(),"l_foot","l_foot_dh_frame");
    }

    KDL::CoDyCo::TreeFkSolverPos_iterative pos_solv(icub_kdl);
    KDL::CoDyCo::GeneralizedJntPositions pos(icub_kdl.getNrOfJoints());

    //Export dh frames (used by iKin and skin software)
    //Before urdf conversion, KDL link frames == iKin link frames
    addFrame(icub_kdl,KDL::Frame::Identity(),"l_forearm","l_forearm_dh_frame");
    addFrame(icub_kdl,KDL::Frame::Identity(),"r_forearm","r_forearm_dh_frame");
    addFrame(icub_kdl,KDL::Frame::Identity(),"l_hand","l_hand_dh_frame");
    addFrame(icub_kdl,KDL::Frame::Identity(),"r_hand","r_hand_dh_frame");
    //expect for link attached to ft sensors, for which KDL link frame == FT sensor frame
    addFrame(icub_kdl,ft_la_H_sensor_dh_child,"l_upper_arm","l_upper_arm_dh_frame");
    addFrame(icub_kdl,ft_ra_H_sensor_dh_child,"r_upper_arm","r_upper_arm_dh_frame");


    KDL::Frame root_link_H_l_foot_dh_frame, root_link_H_l_foot,l_foot_dh_frame_H_l_foot;
    int ret = pos_solv.JntToCart(pos,root_link_H_l_foot_dh_frame,"l_foot_dh_frame");
    yAssert(ret == 0);
    ret = pos_solv.JntToCart(pos,root_link_H_l_foot,"l_foot");
    yAssert(ret == 0);
    l_foot_dh_frame_H_l_foot = root_link_H_l_foot_dh_frame.Inverse()*root_link_H_l_foot;

    KDL::Frame root_link_H_r_foot_dh_frame, root_link_H_r_foot,r_foot_dh_frame_H_r_foot;
    ret = pos_solv.JntToCart(pos,root_link_H_r_foot_dh_frame,"r_foot_dh_frame");
    yAssert(ret == 0);
    ret = pos_solv.JntToCart(pos,root_link_H_r_foot,"r_foot");
    yAssert(ret == 0);
    r_foot_dh_frame_H_r_foot = root_link_H_r_foot_dh_frame.Inverse()*root_link_H_r_foot;


    //Export the world frame used by codyco balancing demo
    //Export l_sole, using the usual origin of the leg end effector and the orientation definde in REP-120
    KDL::Frame l_foot_dh_frame_H_l_sole;
    l_foot_dh_frame_H_l_sole.M = KDL::Rotation (0, 0, 1,
                                               0, -1, 0,
                                                1, 0, 0);

    KDL::Frame l_sole_H_l_foot_dh_frame = l_foot_dh_frame_H_l_sole.Inverse();


    KDL::Frame l_sole_H_l_foot = l_sole_H_l_foot_dh_frame*l_foot_dh_frame_H_l_foot;

    // codyco_balancing_world is X-front, Y-left, Z-up like l_sole and r_sole are defined in REP-120
    addFrame(icub_kdl,l_sole_H_l_foot.Inverse(),"l_foot","codyco_balancing_world");
    addFrame(icub_kdl,l_sole_H_l_foot.Inverse(),"l_foot","l_sole");

    //Export r_sole, using the usual origin of the leg end effector and the orientation definde in REP-120
    KDL::Frame r_foot_dh_frame_H_r_sole;
    r_foot_dh_frame_H_r_sole.M = KDL::Rotation (0, 0, 1,
                                               0, -1, 0,
                                                1, 0, 0);

    KDL::Frame r_sole_H_r_foot_dh_frame = r_foot_dh_frame_H_r_sole.Inverse();


    KDL::Frame r_sole_H_r_foot = r_sole_H_r_foot_dh_frame*r_foot_dh_frame_H_r_foot;

    addFrame(icub_kdl,r_sole_H_r_foot.Inverse(),"r_foot","r_sole");

    // Export chest_skin_frame, see https://github.com/robotology/icub-main/issues/124 for more info
    KDL::Frame root_link_H_chest;
    int retChest = pos_solv.JntToCart(pos,root_link_H_chest,"chest");
    yAssert(retChest == 0);
    KDL::Frame root_link_H_chest_skin_frame;
    iCub::iKin::iCubTorso chest_skin_frame_chest;
    yarp::sig::Vector qTorso(chest_skin_frame_chest.getN());
    qTorso.zero();
    chest_skin_frame_chest.setAng(qTorso);
    yAssert(YarptoKDL(chest_skin_frame_chest.getH(),root_link_H_chest_skin_frame));

    std::cout << "root_link_H_chest_skin_frame is " << root_link_H_chest_skin_frame << std::endl;

    KDL::Frame chest_H_chest_skin_frame = root_link_H_chest.Inverse()*root_link_H_chest_skin_frame;
    addFrame(icub_kdl,chest_H_chest_skin_frame,"chest","chest_skin_frame");


    //std::cout << "Returning from KDL: " << KDL::CoDyCo::UndirectedTree(icub_kdl).toString() << std::endl;

    return true;

}

/*
bool toKDL_iDynDebug(const iCub::iDyn::iCubWholeBody & icub_idyn, KDL::Tree & icub_kdl, bool debug)
{
    bool status_ok = true;
    //Joint names extracted from http://eris.liralab.it/wiki/ICub_joints
    //Serialization: left leg (6), right leg (6), torso (3), left arm (7), right arm (7), head (3).

    //Default "fake" base link for humanoids URDF
    std::string fake_root_name = "base_link";
    icub_kdl = KDL::Tree(fake_root_name);

    std::vector<std::string> joints,links;

    //Creating left leg
    KDL::Chain ll, old_ll;
    const char *ll_joints_cstr[] = {"l_hip_pitch", "l_hip_roll", "l_leg_ft_sensor", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll"};
    std::vector<std::string> ll_joints(ll_joints_cstr,end(ll_joints_cstr));
    names2links_joints(ll_joints,links,joints);
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.lowerTorso->left),*(icub_idyn.lowerTorso->leftSensor),old_ll,links,joints,"l_sole");
    if(!status_ok) return false;


    //Creating right leg
    KDL::Chain rl, old_rl;
    const char *rl_joints_cstr[] = {"r_hip_pitch", "r_hip_roll", "r_leg_ft_sensor", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};
    std::vector<std::string> rl_joints(rl_joints_cstr,end(rl_joints_cstr));
    names2links_joints(rl_joints,links,joints);
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.lowerTorso->right),*(icub_idyn.lowerTorso->rightSensor),old_rl,links,joints,"r_sole");
    if(!status_ok) return false;


    //Creating torso
    KDL::Chain torso, old_torso;
    const char *torso_joints_cstr[] = {"torso_yaw","torso_roll","torso_pitch"};
    std::vector<std::string> torso_joints(torso_joints_cstr,end(torso_joints_cstr));
    names2links_joints(torso_joints,links,joints);
    status_ok = idynChain2kdlChain(*(icub_idyn.lowerTorso->up),old_torso,links,joints);
    if(!status_ok) return false;



    //Creating left arm
    KDL::Chain la, old_la;
    const char *la_joints_cstr[] = {"l_shoulder_pitch", "l_shoulder_roll","l_shoulder_yaw" ,"l_arm_ft_sensor" , "l_elbow", "l_wrist_prosup", "l_wrist_pitch","l_wrist_yaw",};
    std::vector<std::string> la_joints(la_joints_cstr,end(la_joints_cstr));
    names2links_joints(la_joints,links,joints);
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.upperTorso->left),*(icub_idyn.upperTorso->leftSensor),old_la,links,joints,"l_gripper");
    if(!status_ok) return false;


    //Creating right arm
    KDL::Chain ra, old_ra;
    const char *ra_joints_cstr[] = {"r_shoulder_pitch", "r_shoulder_roll","r_shoulder_yaw", "r_arm_ft_sensor", "r_elbow", "r_wrist_prosup", "r_wrist_pitch","r_wrist_yaw",};
    std::vector<std::string> ra_joints(ra_joints_cstr,end(ra_joints_cstr));
    names2links_joints(ra_joints,links,joints);
    status_ok = idynSensorChain2kdlChain(*(icub_idyn.upperTorso->right),*(icub_idyn.upperTorso->rightSensor),old_ra,links,joints,"r_gripper");
    if(!status_ok) return false;


    //Creating head
    KDL::Chain head, old_head;
    const char *head_joints_cstr[] = {"neck_pitch","neck_roll","neck_yaw","imu"};
    std::vector<std::string> head_joints(head_joints_cstr,end(head_joints_cstr));
    names2links_joints(head_joints,links,joints);
    status_ok = idynChain2kdlChain(*(icub_idyn.upperTorso->up),old_head,links,joints);
    if(!status_ok) return false;

    //Now that all the chain are created, it is possible to compose them
    //to create the iCub KDL::Tree

    //First we have to had the root_link, ( for now without RigidBodyInertia!)
    status_ok = icub_kdl.addSegment(KDL::Segment(real_root_name,KDL::Joint("base_joint",KDL::Joint::None),KDL::Frame::Identity()),fake_root_name);
    if(!status_ok) return false;



    //Adding the chains following the serialization
    KDL::Frame kdlFrame;
    idynMatrix2kdlFrame(icub_idyn.lowerTorso->HLeft,kdlFrame);
    addBaseTransformation(old_ll,ll,kdlFrame);

    status_ok = icub_kdl.addChain(ll,real_root_name);
    if(!status_ok) return false;


    idynMatrix2kdlFrame(icub_idyn.lowerTorso->HRight,kdlFrame);
    addBaseTransformation(old_rl,rl,kdlFrame);
    icub_kdl.addChain(rl,real_root_name);

    idynMatrix2kdlFrame(icub_idyn.lowerTorso->HUp,kdlFrame);
    addBaseTransformation(old_torso,torso,kdlFrame);
    icub_kdl.addChain(torso,real_root_name);

    //not using RBT because it is an identity, and it is not clear is
    //semantical meaning (if is H_upper_lower or H_lower_upper )
    idynMatrix2kdlFrame(icub_idyn.upperTorso->HLeft,kdlFrame);
    addBaseTransformation(old_la,la,kdlFrame);
    icub_kdl.addChain(la,arms_head_base_name);

    idynMatrix2kdlFrame(icub_idyn.upperTorso->HRight,kdlFrame);
    //addBaseTransformation(old_ra,ra,kdlFrame);
    status_ok = icub_kdl.addSegment(KDL::Segment("r_torso_debug_link",KDL::Joint("r_torso_debug_joint",KDL::Joint::None),kdlFrame),arms_head_base_name);
    if(!status_ok) return false;

    //icub_kdl.addChain(ra,arms_head_base_name);
    icub_kdl.addChain(old_ra,"r_torso_debug_link");

    idynMatrix2kdlFrame(icub_idyn.upperTorso->HUp,kdlFrame);
    addBaseTransformation(old_head,head,kdlFrame);
    icub_kdl.addChain(head,arms_head_base_name);



    //REP 120
    KDL::Segment kdlSegment = KDL::Segment("torso",KDL::Joint("torso_joint",KDL::Joint::None));
    icub_kdl.addSegment(kdlSegment,arms_head_base_name);



    if( debug ) {
        kdlSegment = KDL::Segment("torso_yaw",KDL::Joint("torso_yaw_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"torso_yaw_link");

        kdlSegment = KDL::Segment("torso_roll",KDL::Joint("torso_roll_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"torso_roll_link");

        kdlSegment = KDL::Segment("torso_pitch",KDL::Joint("torso_pitch_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,arms_head_base_name);


        kdlSegment = KDL::Segment("l_shoulder_pitch",KDL::Joint("l_shoulder_pitch_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"l_shoulder_pitch_link");

        kdlSegment = KDL::Segment("l_shoulder_roll",KDL::Joint("l_shoulder_roll_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"l_shoulder_roll_link");

        kdlSegment = KDL::Segment("l_arm_ft_sensor",KDL::Joint("l_arm_ft_sensor_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"l_arm_ft_sensor_link");


        kdlSegment = KDL::Segment("r_shoulder_pitch",KDL::Joint("r_shoulder_pitch_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_shoulder_pitch_link");

        kdlSegment = KDL::Segment("r_shoulder_roll",KDL::Joint("r_shoulder_roll_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_shoulder_roll_link");

        kdlSegment = KDL::Segment("r_arm_ft_sensor",KDL::Joint("r_arm_ft_sensor_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_arm_ft_sensor_link");

        kdlSegment = KDL::Segment("r_shoulder_yaw",KDL::Joint("r_shoulder_yaw_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_shoulder_yaw_link");

        kdlSegment = KDL::Segment("r_elbow",KDL::Joint("r_elbow_fixed_joint",KDL::Joint::None));
        icub_kdl.addSegment(kdlSegment,"r_elbow_link");

    }

    return true;

}
*/



