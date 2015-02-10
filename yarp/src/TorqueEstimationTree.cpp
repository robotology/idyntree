/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#include "iCub/iDynTree/TorqueEstimationTree.h"
#include "iCub/iDynTree/idyn2kdl_icub.h"

//Urdf import from kdl_format_io
#include <kdl_format_io/urdf_import.hpp>
#include <kdl_format_io/urdf_sensor_import.hpp>

#include <vector>

namespace iCub {
namespace iDynTree {


TorqueEstimationTree::TorqueEstimationTree(std::string urdf_filename,
                                           std::vector<std::string> dof_serialization,
                                           std::vector<std::string> ft_serialization,
                                           std::string fixed_link, unsigned int verbose)
{
    yarp::sig::Vector q_min_yarp, q_max_yarp;

    //Parse a KDL::Tree from URDF
    KDL::Tree icub_kdl;

    bool ret = kdl_format_io::treeFromUrdfFile(urdf_filename,icub_kdl);

    assert(ret);
    if( !ret ) {
        { std::cerr << "[INFO] TorqueEstimationTree: error in costructor" << std::endl; }
        return;
    }

    //Construct F/T sensor name list from URDF gazebo extensions
    std::vector<kdl_format_io::FTSensorData> ft_sensors;
    ret = kdl_format_io::ftSensorsFromUrdfFile(urdf_filename, ft_sensors);

    if( !ret )
    {
        {
            std::cerr << "[ERR] TorqueEstimationTree: error in loading ft_sensors" << std::endl;
            assert(false);
        }
        return;
    }


    int nrOfDofs = dof_serialization.size();
    KDL::JntArray q_min_kdl(nrOfDofs), q_max_kdl(nrOfDofs);
    std::vector<std::string> joint_limits_names;
    kdl_format_io::jointPosLimitsFromUrdfFile(urdf_filename,joint_limits_names,q_min_kdl,q_max_kdl);


    //Set joint limits
    q_min_yarp.resize(nrOfDofs);
    q_max_yarp.resize(nrOfDofs);

    for(int dof = 0; dof < nrOfDofs; dof++ )
    {
        std::string dof_name = dof_serialization[dof];
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

    this->TorqueEstimationConstructor(icub_kdl,ft_sensors,
                                      dof_serialization,ft_serialization,
                                      q_min_yarp,q_max_yarp,fixed_link,verbose);

}

TorqueEstimationTree::TorqueEstimationTree(KDL::Tree& icub_kdl,
                                          std::vector< kdl_format_io::FTSensorData > ft_sensors,
                                          std::vector< std::string > dof_serialization,
                                          std::vector< std::string > ft_serialization,
                                          yarp::sig::Vector& q_min, yarp::sig::Vector& q_max,
                                          std::string fixed_link, unsigned int verbose)
{
    TorqueEstimationConstructor(icub_kdl,ft_sensors,dof_serialization,ft_serialization,q_min,q_max,fixed_link,verbose);
}

TorqueEstimationTree::TorqueEstimationTree(KDL::Tree& icub_kdl,
                                          std::vector< kdl_format_io::FTSensorData > ft_sensors,
                                          std::vector< std::string > ft_serialization,
                                          std::string fixed_link, unsigned int verbose)
{
    yarp::sig::Vector q_max(icub_kdl.getNrOfJoints(),1000.0);
    yarp::sig::Vector q_min(icub_kdl.getNrOfJoints(),-1000.0);

    KDL::CoDyCo::TreeSerialization serial(icub_kdl);

    std::vector<std::string> dof_serialization;

    for(int i = 0; i < serial.getNrOfDOFs(); i++ )
    {
        dof_serialization.push_back(serial.getDOFName(i));
    }

    TorqueEstimationConstructor(icub_kdl,ft_sensors,dof_serialization,ft_serialization,q_min,q_max,fixed_link,verbose);
}



void TorqueEstimationTree::TorqueEstimationConstructor(KDL::Tree & icub_kdl,
                                                  std::vector<kdl_format_io::FTSensorData> ft_sensors,
                                                  std::vector<std::string> dof_serialization,
                                                  std::vector<std::string> ft_serialization,
                                                  yarp::sig::Vector & q_min_yarp, yarp::sig::Vector & q_max_yarp,
                                                  std::string fixed_link, unsigned int verbose)
{
    std::vector< std::string > ft_names(ft_serialization.size());
    std::vector<KDL::Frame> child_sensor_transforms(ft_serialization.size());
    KDL::Frame kdlFrame;

    for(int serialization_id=0; serialization_id < ft_serialization.size(); serialization_id++)
    {
        if( 0 == ft_sensors.size() )
        {
                std::cerr << "[ERR] TorqueEstimationTree: ft sensor " << ft_serialization[serialization_id] << " not found in model file." << std::endl;
                assert(false);
                return;
        }
        for(int ft_sens=0; ft_sens < ft_sensors.size(); ft_sens++ )
        {
            std::string ft_sens_name = ft_sensors[ft_sens].reference_joint;
            int ft_sens_id;

            std::cout << "ft_sens" << ft_sens << std::endl;

            if( ft_serialization[serialization_id] == ft_sens_name)
            {
                std::cout << "Found ft sensor " << ft_sens_name << " in urdf " << std::endl;
                ft_sens_id = serialization_id;

                ft_names[ft_sens_id] = ft_sens_name;
                // \todo TODO FIXME properly address also parent and child cases
                //                  and measure_direction
                if( ft_sensors[ft_sens].frame == kdl_format_io::FTSensorData::SENSOR_FRAME )
                {
                    child_sensor_transforms[ft_sens_id] = KDL::Frame(ft_sensors[ft_sens].sensor_pose.M);
                }
                else
                {
                    child_sensor_transforms[ft_sens_id] = KDL::Frame::Identity();
                }

                break;
            }

            if( ft_sens == ft_sensors.size() -1 )
            {
                std::cerr << "[ERR] TorqueEstimationTree: ft sensor " << ft_sens_name << " not found in model file." << std::endl;
                assert(false);
                return;
            }
        }
    }

        std::cerr << "[INFO] TorqueEstimationTree constructor: loaded urdf with " << dof_serialization.size()
              << "dofs and " << ft_names.size() << " fts ( " << ft_serialization.size() <<  ") " << std::endl;

    //Define an explicit serialization of the links and the DOFs of the iCub
    //The DOF serialization done in icub_kdl construction is ok
    KDL::CoDyCo::TreeSerialization serial = KDL::CoDyCo::TreeSerialization(icub_kdl);

    //Setting a custom dof serialization (\todo TODO FIXME : quite an hack, substitute with proper)
    if( dof_serialization.size() != 0 )
    {
        YARP_ASSERT(dof_serialization.size() == serial.getNrOfDOFs());
        for(int dof=0; dof < dof_serialization.size(); dof++)
        {
            std::string dof_string = dof_serialization[dof];
            YARP_ASSERT(serial.getDOFID(dof_string) != -1);
            YARP_ASSERT(serial.getJunctionID(dof_string) != -1);
        }

        for(int dof=0; dof < dof_serialization.size(); dof++)
        {
            std::string dof_string = dof_serialization[dof];
            std::cout << "[DEBUG] TorqueEstimationTree: Setting id of dof " << dof_string << " to " << dof << std::endl;
            serial.setDOFNameID(dof_string,dof);
            serial.setJunctionNameID(dof_string,dof);
        }
    }

    std::string imu_link_name = "imu_frame";

    if( fixed_link != "" )
    {
        imu_link_name = fixed_link;
    }

    this->constructor(icub_kdl,ft_names,imu_link_name,serial);

    std::cerr << "[INFO] TorqueEstimationTree constructor: loaded urdf with " << this->getNrOfDOFs()
              << "dofs and " << ft_names.size() << " fts ( " << ft_serialization.size() <<  ") " << std::endl;

    assert(this->getNrOfDOFs() > 0);

    this->setJointBoundMin(q_min_yarp);
    this->setJointBoundMax(q_max_yarp);

    return;
}

TorqueEstimationTree::~TorqueEstimationTree() {}

}
}
