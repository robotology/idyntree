/**
* Copyright: 2010-2013 RobotCub Consortium
* Author: Silvio Traversaro, Serena Ivaldi
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
**/

//
// This test/example is based on the iDyn tutorial with the same name,
// to show the similarities in the API between iDyn and iDynTree (and
// for testing
//
// An example on using both iCubTree and iCubWholeBody to estimate the measurements of the FT sensors
// for all the arms and legs, exploiting the modeled dynamic and the inertial sensor
// measurements.
//
// Author: Silvio Traversaro - <silvio.traversaro@iit.it>
// Author: Serena Ivaldi

#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/Time.h>
#include <yarp/os/Random.h>

#include <yarp/math/api.h>

#include <kdl_format_io/urdf_export.hpp>
#include <kdl_format_io/urdf_import.hpp>
#include <kdl_format_io/urdf_sensor_import.hpp>


#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <iCub/iDynTree/TorqueEstimationTree.h>

#include <iCub/iDynTree/idyn2kdl_icub.h>

#include <yarp/os/Log.h>

#include <kdl/frames_io.hpp>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iDyn;
using namespace iCub::iDynTree;


void set_random_vector(yarp::sig::Vector & vec, yarp::os::Random & rng, double coeff=0.0)
{
    for( int i=0; i < (int)vec.size(); i++ ) {
        vec[i] =  coeff*M_PI*rng.uniform();
    }
}

std::vector<std::string> get_iDyn_dof_serialization(KDL::Tree & icub_kdl)
{
    std::vector<std::string> ret;
    KDL::CoDyCo::TreeSerialization serialization(icub_kdl);
    for( int i =0; i < serialization.getNrOfDOFs(); i++ )
    {
        ret.push_back(serialization.getDOFName(i));
    }
    return ret;
}

std::vector<std::string> get_iDyn_ft_serialization()
{
    std::vector<std::string> ret;
    ret.push_back("l_arm_ft_sensor");
    ret.push_back("r_arm_ft_sensor");
    ret.push_back("l_leg_ft_sensor");
    ret.push_back("r_leg_ft_sensor");
    return ret;
}

std::vector<std::string> get_iDyn_ft_frames()
{
    std::vector<std::string> ret;
    ret.push_back("l_arm_ft_frame");
    ret.push_back("r_arm_ft_frame");
    ret.push_back("l_leg_ft_frame");
    ret.push_back("r_leg_ft_frame");
    return ret;
}

std::vector<kdl_format_io::FTSensorData> get_default_ft_sensors(std::vector<std::string> ft_serialization)
{
    std::vector<kdl_format_io::FTSensorData> ret;
    for(int i =0; i < ft_serialization.size(); i++ )
    {
        kdl_format_io::FTSensorData dat;
        dat.reference_joint = ft_serialization[i];
        dat.sensor_name = dat.reference_joint+"_sensor";
        dat.measure_direction = kdl_format_io::FTSensorData::CHILD_TO_PARENT;
        dat.frame = kdl_format_io::FTSensorData::CHILD_LINK_FRAME;
        dat.sensor_pose = KDL::Frame::Identity();

        ret.push_back(dat);
    }

    return ret;
}


struct VectorSlice
{
    unsigned int firstIndex;
    unsigned int length;
};

yarp::sig::Vector getSubVector(std::string part_name,
                               std::map<std::string, VectorSlice> parts,
                               const yarp::sig::Vector vec)
{
    yarp::sig::Vector ret;
    VectorSlice part = parts[part_name];
    ret.resize(part.length);
    for(int i=0; i < part.length; i++)
    {
        ret[i] = vec[part.firstIndex+i];
    }
    return ret;
}

void setSubVector(std::string part_name,
                               std::map<std::string, VectorSlice> parts,
                               yarp::sig::Vector & big_vector,
                               const yarp::sig::Vector & small_vector)
{
    VectorSlice part = parts[part_name];
    YARP_ASSERT(small_vector.size() == part.length);
    for(int i=0; i < part.length; i++)
    {
        big_vector[part.firstIndex+i] = small_vector[i];
    }
    return;
}



std::map<std::string, VectorSlice> get_iDyn_icub_parts()
{
     std::map<std::string, VectorSlice> ret;
    VectorSlice torso_slice, head_slice, left_arm_slice, right_arm_slice, left_leg_slice, right_leg_slice;
    left_leg_slice.firstIndex = 0;
    left_leg_slice.length = 6;
    ret["left_leg"] = left_leg_slice;
    right_leg_slice.firstIndex = 6;
    right_leg_slice.length = 6;
    ret["right_leg"] = right_leg_slice;
    torso_slice.firstIndex = 12;
    torso_slice.length = 3;
    ret["torso"] = torso_slice;
    left_arm_slice.firstIndex  = 15;
    left_arm_slice.length      = 7;
    ret["left_arm"] = left_arm_slice;
    right_arm_slice.firstIndex = 22;
    right_arm_slice.length     = 7;
    ret["right_arm"] = right_arm_slice;
    head_slice.firstIndex = 29;
    head_slice.length  = 3;
    ret["head"] = head_slice;

    return ret;
}

struct consistency_struct
{
    yarp::sig::Vector base_ft;
};

bool checkIdentity(KDL::Frame frame, double tol = 1e-4)
{
    for(int i=0; i < 3; i++ )
    {
       for(int j=0; j < 3; j++ )
       {
          double err;
          if( i == j )
          {
              err = fabs(frame.M(i,j)-1);
          }
          else
          {
              err = fabs(frame.M(i,j));
          }

          if( err < tol ) return false;
       }
    }

    for(int i =0; i < 3; i++ )
    {
        double err = fabs(frame.p[i]);
        if( err < tol ) return false;
    }

    return true;
}

bool printAndCheckIdentity(iCub::iDynTree::TorqueEstimationTree & tree,
                           std::string first_frame,
                           std::string second_frame)
{
    int first_frame_id = tree.getFrameIndex(first_frame);
    int second_frame_id = tree.getFrameIndex(second_frame);
    KDL::Frame pos = tree.getPositionKDL(first_frame_id,second_frame_id);
    std::cout << "Transformation between " << first_frame << " and " << second_frame << " : " << std::endl;
    std::cout << pos << std::endl;
    return checkIdentity(pos);
}

int main()
{
    ////////////////////////////////////////////////////////////////////
    //// iDyn
    ////////////////////////////////////////////////////////////////////
    double tol = 1e-2;


    // declare an icub = head + left arm + right arm + torso + left leg + right leg
    // the kinematic and dynamic parameters of each link are automatically set using the
    // CAD model data.
    // icub default parameters are:
    // mode    = DYNAMIC : the mode for computing wrenches, considering q,dq,d2q,mass,inertia of each link;
    //                     the other main mode is STATIC, which only considers q and mass.
    // verbose = VERBOSE : the verbosity level; I suggest to use VERBOSE during debug, even if you see a lot
    //                     of messages; then to turn it off with NO_VERBOSE only if everything is really fine;
    //                     if you are using iCubWholeBody you probably don't need it a lot because the inner
    //                     classes have been tested first with iCub, but if you use iDyn in general and use
    //                     it on your own code, it's better to turn verbosity on, it may help with the library
    //                     use.
    version_tag ver;
    ver.head_version = 1;
    ver.legs_version = 2;
    iCubWholeBody icub(ver);

    // just priting some information to see how the class works
    // iCubWholeBody is basically a container, with Upper and Lower Torso. these two are
    // in fact two public pointers to the iCubUpperTorso and iCubLowerTorso objects, each
    // having all the useful methods for each limb. To access to a specific limb, you must
    // 'address' it as a string. The string name is quite familiar, it recalls yarp ports..
    cout<<endl
        <<"iCub has many DOF: "<<endl
        <<" - head      : "<<icub.upperTorso->getNLinks("head")<<endl
        <<" - left arm  : "<<icub.upperTorso->getNLinks("left_arm")<<endl
        <<" - right arm : "<<icub.upperTorso->getNLinks("right_arm")<<endl
        <<" - torso     : "<<icub.lowerTorso->getNLinks("torso")<<endl
        <<" - left leg  : "<<icub.lowerTorso->getNLinks("left_leg")<<endl
        <<" - right leg : "<<icub.lowerTorso->getNLinks("right_leg")<<endl<<endl;

    // now we set the joint angles for all the limbs
    // if connected to the real robot, we can take this values from the encoders
    Vector q_head(icub.upperTorso->getNLinks("head"));
    Vector q_larm(icub.upperTorso->getNLinks("left_arm"));
    Vector q_rarm(icub.upperTorso->getNLinks("right_arm"));
    Vector q_torso(icub.lowerTorso->getNLinks("torso"));
    Vector q_lleg(icub.lowerTorso->getNLinks("left_leg"));
    Vector q_rleg(icub.lowerTorso->getNLinks("right_leg"));

    //We can initialize the values to random to test that
    //iDyn and iDynTree return the same results
    yarp::os::Random rng;
    rng.seed((int)yarp::os::Time::now());
    double coeff = 0.0;
    set_random_vector(q_head,rng,0.0);
    set_random_vector(q_larm,rng,0.0);
    set_random_vector(q_rarm,rng,0.0);
    set_random_vector(q_torso,rng,0.0);
    set_random_vector(q_lleg,rng,0.0);
    set_random_vector(q_rleg,rng,0.0);

    // here we set the inertial sensor (head) measurements
    Vector w0(3); Vector dw0(3); Vector ddp0(3);
    w0=dw0=ddp0=0.0; ddp0[2]=9.81;

    // here we set the external forces at the end of each limb
    Matrix FM_up(6,3); FM_up.zero();
    Matrix FM_lo(6,2); FM_lo.zero();

    // here we set the joints position, velocity and acceleration
    // for all the limbs!
    q_head = icub.upperTorso->setAng("head",q_head);
    q_rarm = icub.upperTorso->setAng("right_arm",q_rarm);
    q_larm = icub.upperTorso->setAng("left_arm",q_larm);
    //
    icub.upperTorso->setDAng("head",q_head);
    icub.upperTorso->setDAng("right_arm",q_rarm);
    icub.upperTorso->setDAng("left_arm",q_larm);
    //
    icub.upperTorso->setD2Ang("head",q_head);
    icub.upperTorso->setD2Ang("right_arm",q_rarm);
    icub.upperTorso->setD2Ang("left_arm",q_larm);
    //
    q_torso = icub.lowerTorso->setAng("torso",q_torso);
    q_rleg = icub.lowerTorso->setAng("right_leg",q_rleg);
    q_lleg = icub.lowerTorso->setAng("left_leg",q_lleg);
    //
    icub.lowerTorso->setDAng("torso",q_torso);
    icub.lowerTorso->setDAng("right_leg",q_rleg);
    icub.lowerTorso->setDAng("left_leg",q_lleg);
    //
    icub.lowerTorso->setD2Ang("torso",q_torso);
    icub.lowerTorso->setD2Ang("right_leg",q_rleg);
    icub.lowerTorso->setD2Ang("left_leg",q_lleg);

    // initialize the head with the kinematic measures retrieved
    // by the inertial sensor on the head
    icub.upperTorso->setInertialMeasure(w0,dw0,ddp0);


    // solve the kinematic of all the limbs attached to the upper torso
    // i.e. head, right arm and left arm
    // then solve the wrench phase, where the external wrenches are set
    // to be acting only on the end-effector of each limb
    // then the limbs with a FT sensor (arms) estimate the sensor wrench
    // note: FM_up = 6x3, because we have 3 limbs with external wrench input
    // note: afterAttach=false is default
    Matrix fm_sens_up = icub.upperTorso->estimateSensorsWrench(FM_up,false);

    // connect upper and lower torso: they exchange kinematic and wrench information
    icub.lowerTorso->setInertialMeasure(icub.upperTorso->getTorsoAngVel(),icub.upperTorso->getTorsoAngAcc(),icub.upperTorso->getTorsoLinAcc());



    // now solve lower torso the same way of the upper
    // note: FM_lo = 6x2, because we have 2 limbs with external wrench input to be set
    // since the torso receives the external wrench output from the upperTorso node
    // during the attachTorso()
    // note: afterAttach=false is set to true, because we specify that the torso
    // already received the wrench for initialization by the upperTorso during
    // the attachTorso()
    Matrix fm_sens_lo = icub.lowerTorso->estimateSensorsWrench(FM_lo);

    cout<<endl
        <<"Estimate FT sensor measurements on upper body: "<<endl
        <<" left  : "<<fm_sens_up.getCol(1).toString()<<endl
        <<" right : "<<fm_sens_up.getCol(0).toString()<<endl
        <<endl
        <<"Estimate FT sensor measurements on lower body: "<<endl
        <<" left  : "<<fm_sens_lo.getCol(1).toString()<<endl
        <<" right : "<<fm_sens_lo.getCol(0).toString()<<endl
        <<endl
        <<"Torques in upper body" << endl
        <<" head : " << icub.upperTorso->getTorques("head").toString() << endl
        <<" left arm: " << icub.upperTorso->getTorques("left_arm").toString() << endl
        <<" right arm: " << icub.upperTorso->getTorques("right_arm").toString() << endl
        <<"Torques in lower body" << endl
        <<" torso : " << icub.lowerTorso->getTorques("torso").toString() << endl
        <<" left leg: " << icub.lowerTorso->getTorques("left_leg").toString() << endl
        <<" right leg: " << icub.lowerTorso->getTorques("right_leg").toString() << endl;


    ////////////////////////////////////////////////////////////////////
    //// iDynTree
    ////////////////////////////////////////////////////////////////////

        KDL::JntArray q_min,q_max;
    std::vector<std::string> dof_serialization;

    std::vector<consistency_struct> consistency_data(2);

    for(int consistency_test=0; consistency_test <= 1; consistency_test++ )
    {

    //Do the tests two times: first (for consistency_test = 0) we export a kdl tree
    // from iDyn, and we test it, then we export it to urdf (consistency_test = 1)
    // and we reimport the exported urdf


    std::string urdf_filename = "urdf_icub_test.urdf";
            KDL::Tree icub_kdl;

    if( consistency_test == 0 )
    {
        toKDL(icub,icub_kdl,q_min,q_max,iCub::iDynTree::IDYN_SERIALIZATION);

        //Export to urdf for subsequent test
        boost::shared_ptr<urdf::ModelInterface> icub_ptr(new urdf::ModelInterface);

        if( ! kdl_format_io::treeToUrdfModel(icub_kdl,"test_icub",*icub_ptr) ) {
            std::cerr << "Fatal error in KDL - URDF conversion" << std::endl;
            return EXIT_FAILURE;
        }

        TiXmlDocument* xml_doc = urdf::exportURDF(icub_ptr);
        if( ! xml_doc->SaveFile(urdf_filename) ) {
            std::cerr << "Fatal error in URDF xml saving" << std::endl;
        }

        dof_serialization = get_iDyn_dof_serialization(icub_kdl);

        //Check that the new ft sensor frame added are consistent with the old model (i.e.
        // they should be solidal with the parent link of the FT sensor frame)



    } else
    {
        if( ! kdl_format_io::treeFromUrdfFile(urdf_filename,icub_kdl) ) {
            std::cerr << "Fatal error in URDF ---> conversion" << std::endl;
            return EXIT_FAILURE;
        }
    }

    // declare an iCubTree

    //Convert the iDyn model to a KDL::Tree,
    //Use the same serialization as used in IDYN_SERIALIZATION
    yarp::sig::Vector q_min_yarp(q_min.rows()), q_max_yarp(q_max.rows());


    std::vector<std::string> ft_serialization = get_iDyn_ft_serialization();
    //std::vector<std::string> ft_frames        = get_iDyn_ft_frames();

    std::map<std::string, VectorSlice> icub_parts = get_iDyn_icub_parts();
    std::vector<kdl_format_io::FTSensorData> ft_sensors_data;

    ft_sensors_data = get_default_ft_sensors(ft_serialization);

    for(int i =0; i < q_min_yarp.size(); i++ )
    {
       q_min_yarp[i] = q_min(i);
       q_max_yarp[i] = q_max(i);
    }

    TorqueEstimationTree icub_tree(icub_kdl,
                                   ft_sensors_data,
                                   dof_serialization,
                                   ft_serialization,
                                   q_min_yarp,
                                   q_max_yarp,
                                   "imu_frame",0);

    if( consistency_test == 0 )
    {
        // if dealing with the model directly converted from iDyn, do
        // a consistency check on ft sensor frames
        /*
        printAndCheckIdentity(icub_tree,"l_arm_ft_frame","l_shoulder_3");
        printAndCheckIdentity(icub_tree,"r_arm_ft_frame","r_shoulder_3");
        printAndCheckIdentity(icub_tree,"l_leg_ft_frame","l_hip_2");
        printAndCheckIdentity(icub_tree,"r_leg_ft_frame","r_hip_2");
        */
    }

    // here we set the joints position, velocity and acceleration
    // for all the limbs!
    yarp::sig::Vector qj_all(32,0.0), dqj_all(32,0.0), ddqj_all(32,0.0);


    setSubVector("head",icub_parts,qj_all,q_head);
    setSubVector("right_arm",icub_parts,qj_all,q_rarm);
    setSubVector("left_arm",icub_parts,qj_all,q_larm);
    setSubVector("torso",icub_parts,qj_all,q_torso);
    setSubVector("right_leg",icub_parts,qj_all,q_rleg);
    setSubVector("left_leg",icub_parts,qj_all,q_lleg);

    icub_tree.setAng(qj_all);

    setSubVector("head",icub_parts,dqj_all,q_head);
    setSubVector("right_arm",icub_parts,dqj_all,q_rarm);
    setSubVector("left_arm",icub_parts,dqj_all,q_larm);
    setSubVector("torso",icub_parts,dqj_all,q_torso);
    setSubVector("right_leg",icub_parts,dqj_all,q_rleg);
    setSubVector("left_leg",icub_parts,dqj_all,q_lleg);

    icub_tree.setDAng(dqj_all);

    setSubVector("head",icub_parts,ddqj_all,q_head);
    setSubVector("right_arm",icub_parts,ddqj_all,q_rarm);
    setSubVector("left_arm",icub_parts,ddqj_all,q_larm);
    setSubVector("torso",icub_parts,ddqj_all,q_torso);
    setSubVector("right_leg",icub_parts,ddqj_all,q_rleg);
    setSubVector("left_leg",icub_parts,ddqj_all,q_lleg);

    icub_tree.setD2Ang(ddqj_all);



    // initialize the head with the kinematic measures retrieved
    // by the inertial sensor on the head
    icub_tree.setInertialMeasure(w0,dw0,ddp0);

    icub_tree.kinematicRNEA();
    icub_tree.dynamicRNEA();

    Vector left_arm_ft(6), right_arm_ft(6), left_leg_ft(6), right_leg_ft(6);
    icub_tree.getSensorMeasurement(0,left_arm_ft);
    icub_tree.getSensorMeasurement(1,right_arm_ft);
    icub_tree.getSensorMeasurement(2,left_leg_ft);
    icub_tree.getSensorMeasurement(3,right_leg_ft);

    Vector com(3);
    double mass;
    icub.computeCOM();
    icub.getCOM(iCub::skinDynLib::BODY_PART_ALL,com,mass);

    cout<<endl
    <<"Estimate FT sensor measurements on upper body: "<<endl
    <<" left  : "<<left_arm_ft.toString()<<endl
    <<" idyn  : "<<fm_sens_up.getCol(1).toString()<<endl
    <<" right : "<<right_arm_ft.toString()<<endl
    <<" idyn  : "<<fm_sens_up.getCol(0).toString()<<endl
    <<endl
    <<"Estimate FT sensor measurements on lower body: "<<endl
    <<" left  : "<<left_leg_ft.toString()<<endl
    <<" idyn  : "<<fm_sens_lo.getCol(1).toString()<<endl
    <<" right : "<<right_leg_ft.toString()<<endl
    <<" idyn  : "<<fm_sens_lo.getCol(0).toString()<<endl
    <<endl;

    consistency_data[consistency_test].base_ft = icub_tree.getBaseForceTorque();


    cout<<"Torques in upper body" << endl
    <<" head : " << getSubVector("head",icub_parts,icub_tree.getTorques()).toString() << endl
    <<" left arm: " << getSubVector("left_arm",icub_parts,icub_tree.getTorques()).toString() << endl
    <<" right arm: " << getSubVector("right_arm",icub_parts,icub_tree.getTorques()).toString() << endl
    <<"Torques in lower body" << endl
    <<" torso : " <<  getSubVector("torso",icub_parts,icub_tree.getTorques()).toString() << endl
    <<" left leg: " <<  getSubVector("left_leg",icub_parts,icub_tree.getTorques()).toString() << endl
    <<" right leg: " <<  getSubVector("right_leg",icub_parts,icub_tree.getTorques()).toString() << endl;



    cout <<"Difference in estimate FT sensor measurements on upper body (iDyn return the force of the parent on the child, while iDynTree and the real robot the one of the child on the parent): "<<endl
    <<" left  : "<<(fm_sens_up.getCol(1) +left_arm_ft).toString()<<endl
    <<" right : "<<(fm_sens_up.getCol(0) +right_arm_ft).toString()<<endl
    <<endl
    <<"Difference in estimate FT sensor measurements on lower body: "<<endl
    <<" left  : "<<(fm_sens_lo.getCol(1)+left_leg_ft).toString()<<endl
    <<" right : "<<(fm_sens_lo.getCol(0)+right_leg_ft).toString()<<endl
    <<endl;


    cout
    <<"Difference in torques in upper body" << endl
    <<" head : " << (icub.upperTorso->getTorques("head").subVector(0,2)-getSubVector("head",icub_parts,icub_tree.getTorques())).toString() << endl
    <<" left arm: " << (icub.upperTorso->getTorques("left_arm")-getSubVector("left_arm",icub_parts,icub_tree.getTorques())).toString() << endl
    <<" right arm: " << (icub.upperTorso->getTorques("right_arm")-getSubVector("right_arm",icub_parts,icub_tree.getTorques())).toString() << endl
    <<"Difference in torques in lower body" << endl
    <<" torso : " << (icub.lowerTorso->getTorques("torso")-getSubVector("torso",icub_parts,icub_tree.getTorques())).toString() << endl
    <<" left leg: " <<  (icub.lowerTorso->getTorques("left_leg")-getSubVector("left_leg",icub_parts,icub_tree.getTorques())).toString() << endl
    <<" right leg: " <<  (icub.lowerTorso->getTorques("right_leg")-getSubVector("right_leg",icub_parts,icub_tree.getTorques())).toString() << endl;


    yarp::sig::Vector head_trq = getSubVector("head",icub_parts,icub_tree.getTorques());
    yarp::sig::Vector torso_trq = getSubVector("torso",icub_parts,icub_tree.getTorques());
    yarp::sig::Vector left_arm_trq = getSubVector("left_arm",icub_parts,icub_tree.getTorques());
    yarp::sig::Vector right_arm_trq = getSubVector("right_arm",icub_parts,icub_tree.getTorques());
    yarp::sig::Vector left_leg_trq = getSubVector("left_leg",icub_parts,icub_tree.getTorques());
    yarp::sig::Vector right_leg_trq = getSubVector("right_leg",icub_parts,icub_tree.getTorques());

    yarp::sig::Vector head_pos = getSubVector("head",icub_parts,icub_tree.getAng());
    yarp::sig::Vector torso_pos = getSubVector("torso",icub_parts,icub_tree.getAng());
    yarp::sig::Vector left_arm_pos = getSubVector("left_arm",icub_parts,icub_tree.getAng());
    yarp::sig::Vector right_arm_pos = getSubVector("right_arm",icub_parts,icub_tree.getAng());
    yarp::sig::Vector left_leg_pos = getSubVector("left_leg",icub_parts,icub_tree.getAng());
    yarp::sig::Vector right_leg_pos = getSubVector("right_leg",icub_parts,icub_tree.getAng());


    cout <<"Difference in positions in upper body" << endl
    <<" head : " << (icub.upperTorso->getAng("head").subVector(0,2)-head_pos).toString() << endl
    <<" left arm: " << (icub.upperTorso->getAng("left_arm")-left_arm_pos).toString() << endl
    <<" right arm: " << (icub.upperTorso->getAng("right_arm")-right_arm_pos).toString() << endl
    <<"Difference in positioon in lower body" << endl
    <<" torso : " << (icub.lowerTorso->getAng("torso")-torso_pos).toString() << endl
    <<" left leg: " <<  (icub.lowerTorso->getAng("left_leg")-left_leg_pos).toString() << endl
    <<" right leg: " <<  (icub.lowerTorso->getAng("right_leg")-right_leg_pos).toString() << endl

    << "Difference in COM " << (com-icub_tree.getCOM()).toString() << std::endl
    << "Original COM " << com.toString() << std::endl
    << "iDynTree COM " << ((icub_tree.getCOM())).toString() << std::endl;


    if( norm(fm_sens_up.getCol(1) +left_arm_ft) > tol ) { return EXIT_FAILURE; }
    if( norm(fm_sens_up.getCol(0) +right_arm_ft) > tol ) { return EXIT_FAILURE; }
    if( norm((fm_sens_lo.getCol(1)+left_leg_ft)) > tol ) { return EXIT_FAILURE; }
    if( norm(fm_sens_lo.getCol(0)+right_leg_ft) > tol ) { return EXIT_FAILURE; }


    if( norm(icub.upperTorso->getTorques("head").subVector(0,2)-head_trq) > tol )
    {
        return EXIT_FAILURE;
    }

    if( norm(icub.upperTorso->getTorques("left_arm")-left_arm_trq) > tol ) { return EXIT_FAILURE; }
    if( norm(icub.upperTorso->getTorques("right_arm")-right_arm_trq) > tol ) { return EXIT_FAILURE; }
    if( norm(icub.lowerTorso->getTorques("left_leg")-left_leg_trq) > tol ) { return EXIT_FAILURE; }
    if( norm(icub.lowerTorso->getTorques("right_leg")-right_leg_trq) > tol ) { return EXIT_FAILURE; }
    //if( norm(icub.lowerTorso->getTorques("torso")-torso_trq) > tol ) { return EXIT_FAILURE; }

    if( norm(icub.upperTorso->getAng("head").subVector(0,2)-head_pos) > tol ) { return EXIT_FAILURE; }
    if( norm(icub.upperTorso->getAng("left_arm")-left_arm_pos) > tol ) { return EXIT_FAILURE; }
    if( norm(icub.upperTorso->getAng("right_arm")-right_arm_pos) > tol ) { return EXIT_FAILURE; }
    if( norm(icub.lowerTorso->getAng("torso")-torso_pos) > tol ) { return EXIT_FAILURE; }
    if( norm(icub.lowerTorso->getAng("left_leg")-left_leg_pos) > tol ) { return EXIT_FAILURE; }
    if( norm(icub.lowerTorso->getAng("right_leg")-right_leg_pos) > tol ) { return EXIT_FAILURE; }

    if( norm(com-icub_tree.getCOM()) > tol ) { return EXIT_FAILURE; }

    }

    //consistency tests between original and converted to urdf
    std::cout << "Base force torque: " << std::endl;
    std::cout << " original : " << consistency_data[0].base_ft.toString() << std::endl;
    std::cout << " urdf     : " << consistency_data[1].base_ft.toString() << std::endl;

    return EXIT_SUCCESS;
}



