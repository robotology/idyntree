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

#include <yarp/os/Log.h>

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

#include <kdl_codyco/treefksolverpos_iterative.hpp>

#include <kdl/frames_io.hpp>

#include <kdl_codyco/config.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iDyn;
using namespace iCub::iDynTree;


void set_random_vector(yarp::sig::Vector & vec, yarp::os::Random & rng, double coeff=1.0)
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

std::vector<std::string> getLinkAttachedToFixedJoints(KDL::Tree & tree)
{
    std::vector<std::string> ret;
    for(KDL::SegmentMap::const_iterator seg=tree.getSegments().begin();
        seg != tree.getSegments().end();
        seg++)
    {
        if( GetTreeElementSegment(seg->second).getJoint().getType() == KDL::Joint::None )
        {
            ret.push_back(GetTreeElementSegment(seg->second).getName());
        }
    }
    return ret;
}

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

    ////////////////////////////////////////////////////////////////////
    //// iDynTree
    ////////////////////////////////////////////////////////////////////

    KDL::JntArray q_min,q_max;
    std::vector<std::string> dof_serialization;

    //Do the tests two times: first (for consistency_test = 0) we export a kdl tree
    // from iDyn, and we test it, then we export it to urdf (consistency_test = 1)
    // and we reimport the exported urdf


     std::string urdf_filename = "urdf_icub_test.urdf";
     KDL::Tree icub_kdl;
     KDL::Tree icub_kdl_urdf;

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



        if( ! kdl_format_io::treeFromUrdfFile(urdf_filename,icub_kdl_urdf) ) {
            std::cerr << "Fatal error in URDF ---> conversion" << std::endl;
            return EXIT_FAILURE;
        }

    KDL::CoDyCo::TreeSerialization serial(icub_kdl);

    //Check that positions of frames of links attached to fixed joints are consistent
    // (they should not be modified by the urdf export routine
    std::vector<std::string> links_to_check = getLinkAttachedToFixedJoints(icub_kdl);
    std::vector<std::string> links_to_check_urdf = getLinkAttachedToFixedJoints(icub_kdl);

    if( links_to_check.size() != links_to_check_urdf.size() )
    {
        std::cerr << "Error links_to_check mismatch" << std::endl;
        return EXIT_FAILURE;
    }

    KDL::JntArray qj(serial.getNrOfDOFs());

    //SetToZero(qj);

    for(int i = 0; i < qj.rows(); i++ )
    {
        qj(i) = i/2.0;
    }

    KDL::CoDyCo::GeneralizedJntPositions q(KDL::Frame::Identity(),qj);

    KDL::CoDyCo::TreeFkSolverPos_iterative pos_solver_icub(icub_kdl,serial);

    KDL::CoDyCo::TreeFkSolverPos_iterative pos_solver_icub_urdf(icub_kdl_urdf,serial);

    for(int i = 0; i < links_to_check.size(); i++ )
    {
       KDL::Frame original_trans, urdf_trans;
       std::string link = links_to_check[i];
       pos_solver_icub.JntToCart(q,original_trans,link);
       pos_solver_icub_urdf.JntToCart(q,urdf_trans,link);

       std::cout << "Base --> Link transformation for link: " << link <<  std::endl;

       std::cout << "Original: " << std::endl;
       std::cout << original_trans << std::endl;

       std::cout << "URDF: " << std::endl;
       std::cout << urdf_trans << std::endl;

       std::cout << "Identity check" << std::endl;
       std::cout << urdf_trans.Inverse()*original_trans << std::endl;

       if( checkIdentity(urdf_trans.Inverse()*original_trans) )
       {
           std::cout << "Original and identity transformation don't match." << std::endl;
           return EXIT_FAILURE;
       }
    }


    return EXIT_SUCCESS;
}



