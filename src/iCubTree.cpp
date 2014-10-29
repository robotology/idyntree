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

//Urdf import from kdl_format_io
#include <kdl_format_io/urdf_import.hpp>
#include <kdl_format_io/urdf_sensor_import.hpp>

namespace iCub {
namespace iDynTree {


iCubTree::iCubTree(std::string urdf_filename, std::string fixed_link, unsigned int verbose)
{
    yarp::sig::Vector q_min_yarp, q_max_yarp;

    //Convert it to a KDL::Tree (this preserve all the frame of reference, is the conversion to URDF that changes them)
    KDL::Tree icub_kdl;

    bool ret = kdl_format_io::treeFromUrdfFile(urdf_filename,icub_kdl);

    assert(ret);
    if( !ret ) {
        { std::cerr << "[INFO] iCubTree: error in costructor" << std::endl; }
        return;
    }

    //Construct F/T sensor name list
    std::vector<kdl_format_io::FTSensorData> ft_sensors;
    ret = kdl_format_io::ftSensorsFromUrdfFile(urdf_filename, ft_sensors);

    if( !ret ) {
        { std::cerr << "[INFO] iCubTree: error in loading ft_sensors" << std::endl; }
        return;
    }


    std::vector< std::string > ft_names(0);
    std::vector<KDL::Frame> child_sensor_transforms(0);
    KDL::Frame kdlFrame;

    for(int ft_sens=0; ft_sens < ft_sensors.size(); ft_sens++ )
    {
        ft_names.push_back(ft_sensors[ft_sens].reference_joint);
        // \todo TODO FIXME properly address also parent and child cases
        //                  and measure_direction
        if( ft_sensors[ft_sens].frame == kdl_format_io::FTSensorData::SENSOR_FRAME )
        {
            child_sensor_transforms.push_back(KDL::Frame(ft_sensors[ft_sens].sensor_pose.M));
        }
        else
        {
            child_sensor_transforms.push_back(KDL::Frame::Identity());
        }
    }

    //Define an explicit serialization of the links and the DOFs of the iCub
    //The DOF serialization done in icub_kdl construction is ok
    KDL::CoDyCo::TreeSerialization serial = KDL::CoDyCo::TreeSerialization(icub_kdl);

    std::string imu_link_name = "imu_frame";

    if( fixed_link != "" )
    {
        imu_link_name = fixed_link;
    }

    this->constructor(icub_kdl,ft_names,imu_link_name,serial);

    std::cout << "[INFO] iCubTree constructor: loaded urdf with " << this->getNrOfDOFs() << "dofs" << std::endl;
    
    assert(this->getNrOfDOFs() > 0);

    //Set joint limits
    KDL::JntArray q_min_kdl(serial.getNrOfDOFs()), q_max_kdl(serial.getNrOfDOFs());
    std::vector<std::string> joint_limits_names;
    kdl_format_io::jointPosLimitsFromUrdfFile(urdf_filename,joint_limits_names,q_min_kdl,q_max_kdl);

    q_min_yarp.resize(serial.getNrOfDOFs());
    q_max_yarp.resize(serial.getNrOfDOFs());

    for(int dof = 0; dof < serial.getNrOfDOFs(); dof++ )
    {
        std::string dof_name = serial.getDOFName(dof);
        for(int lim = 0; lim < joint_limits_names.size(); lim++ )
        {
            if( joint_limits_names[lim] == dof_name )
            {
                q_min_yarp[dof] = q_min_kdl(lim);
                q_max_yarp[dof] = q_max_yarp(lim);
                break;
            }
        }
    }

    this->setJointBoundMin(q_min_yarp);
    this->setJointBoundMax(q_max_yarp);

    return;
}

iCubTree::~iCubTree() {}

}
}
