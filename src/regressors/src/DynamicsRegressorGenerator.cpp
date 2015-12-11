/**
 * Copyright (C) 2015 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#include "iDynTree/Regressors/DynamicsRegressorGenerator.h"

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Rotation.h>


#include "kdl_codyco/regressors/dynamicRegressorGenerator.hpp"
#include <kdl_codyco/KDLConversions.h>

#include <iDynTree/ModelIO/impl/urdf_import.hpp>
#include <iDynTree/ModelIO/impl/urdf_sensor_import.hpp>

#include <kdl/frames.hpp>

#include <tinyxml.h>

#include <iostream>
#include <fstream>

namespace iDynTree
{

namespace Regressors
{

struct DynamicsRegressorGenerator::DynamicsRegressorGeneratorPrivateAttributes
{
    bool m_isRegressorValid;
    bool m_isModelValid;
    KDL::CoDyCo::UndirectedTree robot_model;
    iDynTree::SensorsList sensors_model;
    KDL::CoDyCo::Regressors::DynamicRegressorGenerator * m_pLegacyGenerator;
    Eigen::MatrixXd m_regressor;
    Eigen::VectorXd m_knownTerms;
    Eigen::VectorXd m_parameters;
    Eigen::MatrixXd m_basisMatrix;
    KDL::JntArray m_qKDL;
    KDL::JntArray m_dqKDL;
    KDL::JntArray m_ddqKDL;

    DynamicsRegressorGeneratorPrivateAttributes()
    {
        m_isModelValid = false;
        m_isModelValid = false;
        m_pLegacyGenerator = 0;
    }
};

DynamicsRegressorGenerator::DynamicsRegressorGenerator():
pimpl(new DynamicsRegressorGeneratorPrivateAttributes)
{
}

DynamicsRegressorGenerator::DynamicsRegressorGenerator(const DynamicsRegressorGenerator & other):
pimpl(new DynamicsRegressorGeneratorPrivateAttributes(*(other.pimpl)))
{
    // copyng the class is disabled until we get rid of the legacy implementation
    assert(false);
}

DynamicsRegressorGenerator& DynamicsRegressorGenerator::operator=(const DynamicsRegressorGenerator& other)
{
    /*
    if(this != &other)
    {
        *pimpl = *(other.pimpl);
    }
    return *this;
    */
    // copyng the class is disable until we get rid of the legacy implementation
    assert(false);

    return *this;
}

DynamicsRegressorGenerator::~DynamicsRegressorGenerator()
{
    delete this->pimpl->m_pLegacyGenerator;
    delete this->pimpl;
}

bool DynamicsRegressorGenerator::loadRobotAndSensorsModelFromFile(const std::string& filename,
                                                                  const std::string& filetype)
{
    if( filetype != "urdf" )
    {
        std::cerr << "[ERROR] unknown format " << filetype <<
                     " . Currently only the urdf format is supported." << std::endl;
        return false;
    }

    std::ifstream ifs(filename.c_str());

    if( !ifs )
    {
        std::cerr << "[ERROR] impossible to open file " << filename << std::endl;
        return false;
    }

    std::string model_string( (std::istreambuf_iterator<char>(ifs) ),
                            (std::istreambuf_iterator<char>()    ) );

    return this->loadRobotAndSensorsModelFromString(model_string);
}

bool DynamicsRegressorGenerator::loadRobotAndSensorsModelFromString(const std::string& modelString,
                                                                    const std::string& filetype)
{
    if( filetype != "urdf" )
    {
        std::cerr << "[ERROR] unknown format " << filetype <<
                     " . Currently only the urdf format is supported." << std::endl;
        return false;
    }

    bool consider_root_link_inertia = false;
    KDL::Tree local_model;
    bool ok = iDynTree::treeFromUrdfString(modelString,local_model,consider_root_link_inertia);


    this->pimpl->robot_model = KDL::CoDyCo::UndirectedTree(local_model);
    this->pimpl->sensors_model = iDynTree::sensorsListFromURDFString(this->pimpl->robot_model,
                                                                          modelString);

    if( !ok )
    {
        std::cerr << "[ERROR] error in loading robot model" << std::endl;
        return false;
    }
    else
    {
        this->pimpl->m_isModelValid = true;
        return true;
    }
}


bool DynamicsRegressorGenerator::loadRegressorStructureFromFile(const std::string& filename)
{
    if( !(pimpl->m_isModelValid) )
    {
        std::cerr << "[ERROR] please load a valid mode in DynamicRegressorGenerator before tryng"
                     " to load a regressor structure" << std::endl;
        return false;
    }

    std::ifstream ifs(filename.c_str());

    if( !ifs )
    {
        std::cerr << "[ERROR] impossible to open file " << filename << std::endl;
        return false;
    }


    std::string model_string( (std::istreambuf_iterator<char>(ifs) ),
                              (std::istreambuf_iterator<char>()    ) );

    return this->loadRegressorStructureFromString(model_string);
}

bool DynamicsRegressorGenerator::loadRegressorStructureFromString(const std::string& regressorStructureString)
{
    if( !(pimpl->m_isModelValid) )
    {
        std::cerr << "[ERROR] please load a valid mode in DynamicRegressorGenerator before tryng"
                     " to load a regressor structure" << std::endl;
        return false;
    }

    // if a regressor was already loaded, properly delete it
    if( pimpl->m_pLegacyGenerator != 0 )
    {
        delete pimpl->m_pLegacyGenerator;
        pimpl->m_pLegacyGenerator = 0;
    }

    TiXmlDocument urdfXml;
    urdfXml.Parse(regressorStructureString.c_str());

    // Get root element
    TiXmlElement* regressorXml = urdfXml.FirstChildElement("regressor");

    // Allocate the legacy class
    std::string kinematic_base = "";
    bool consider_ft_offset = true;
    std::vector< std::string > ignoredLinks;
    std::vector< std::string > dummy;

    // We remove the fake links that in the urdf
    // we use as surrogate for frames
    iDynTree::framesFromKDLTree(this->pimpl->robot_model.getTree(),
                                     ignoredLinks,dummy);

    // Add ignored links to the list of links consired "fake"
    for (TiXmlElement* ignoredLinkXml = regressorXml->FirstChildElement("ignoredLink");
         ignoredLinkXml; ignoredLinkXml = ignoredLinkXml->NextSiblingElement("ignoredLink"))
    {
        std::string ignoredLinkName = ignoredLinkXml->GetText();
        ignoredLinks.push_back(ignoredLinkName);
    }

    bool verbose = true;
    this->pimpl->m_pLegacyGenerator =
        new KDL::CoDyCo::Regressors::DynamicRegressorGenerator(this->pimpl->robot_model,
                                                           this->pimpl->sensors_model,
                                                           kinematic_base,
                                                           consider_ft_offset,
                                                           ignoredLinks,verbose);

    // Get all subregressors of type subtreeBaseDynamics
    // For each subtreeBaseDynamics subregressor, add it to the legacy class
    for (TiXmlElement* subtreeBaseDynamicsXml = regressorXml->FirstChildElement("subtreeBaseDynamics");
         subtreeBaseDynamicsXml; subtreeBaseDynamicsXml = subtreeBaseDynamicsXml->NextSiblingElement("subtreeBaseDynamics"))
    {
        // For each subtreeBaseDynamics subregressor, add it to the legacy class
        std::vector<std::string> leaf_links;
        for (TiXmlElement* FTSensorLinkXml = subtreeBaseDynamicsXml->FirstChildElement("FTSensorLink");
            FTSensorLinkXml; FTSensorLinkXml = subtreeBaseDynamicsXml->NextSiblingElement("FTSensorLink"))
        {
            std::string leaf_link = FTSensorLinkXml->GetText();
            leaf_links.push_back(leaf_link);
        }

        if( this->pimpl->m_pLegacyGenerator->addSubtreeRegressorRows(leaf_links) != 0 )
        {
            std::cerr << "[ERROR] error in loading subtreeBaseDynamics regressor" << std::endl;
            delete this->pimpl->m_pLegacyGenerator;
            this->pimpl->m_pLegacyGenerator = 0;
            return false;
        }
    }

    bool ok = this->pimpl->m_pLegacyGenerator->configure();

    if( !ok )
    {
         std::cerr << "[ERROR] error in configure of the regressor" << std::endl;
         delete this->pimpl->m_pLegacyGenerator;
         this->pimpl->m_pLegacyGenerator = 0;
         return false;
    }

    // update buffers
    this->pimpl->m_regressor.resize(this->getNrOfOutputs(),this->getNrOfParameters());
    this->pimpl->m_knownTerms.resize(this->getNrOfOutputs());
    this->pimpl->m_parameters.resize(this->getNrOfParameters());
    this->pimpl->m_qKDL.resize(this->getNrOfDegreesOfFreedom());
    this->pimpl->m_dqKDL.resize(this->getNrOfDegreesOfFreedom());
    this->pimpl->m_ddqKDL.resize(this->getNrOfDegreesOfFreedom());


    this->pimpl->m_isRegressorValid = true;

    return true;
}


bool DynamicsRegressorGenerator::isValid()
{
    return (this->pimpl->m_isModelValid) && (this->pimpl->m_isRegressorValid);
}

const SensorsList& DynamicsRegressorGenerator::getSensorsModel() const
{
    return this->pimpl->m_pLegacyGenerator->sensorsList;
}

std::string DynamicsRegressorGenerator::getBaseLinkName()
{
    int base_link = this->pimpl->m_pLegacyGenerator->getDynamicBaseIndex();
    return this->pimpl->robot_model.getLink(base_link)->getName();
}

//////////////////////////////////////////////////////////////////////////////
//// Output related methods
//////////////////////////////////////////////////////////////////////////////

unsigned int DynamicsRegressorGenerator::getNrOfOutputs() const
{
    return (unsigned int)this->pimpl->m_pLegacyGenerator->getNrOfOutputs();
}

std::string DynamicsRegressorGenerator::getDescriptionOfOutput(int output_index)
{
    return this->pimpl->m_pLegacyGenerator->getDescriptionOfOutput(output_index);
}

std::string DynamicsRegressorGenerator::getDescriptionOfOutputs()
{
    return this->pimpl->m_pLegacyGenerator->getDescriptionOfOutputs();
}

//////////////////////////////////////////////////////////////////////////////
//// Parameters related methods
//////////////////////////////////////////////////////////////////////////////

unsigned int DynamicsRegressorGenerator::getNrOfParameters() const
{
    return (unsigned int)this->pimpl->m_pLegacyGenerator->getNrOfParameters();
}

std::string DynamicsRegressorGenerator::getDescriptionOfParameter(int parameter_index, bool with_value, double value)
{
    return this->pimpl->m_pLegacyGenerator->getDescriptionOfParameter(parameter_index,with_value,value);
}

std::string DynamicsRegressorGenerator::getDescriptionOfParameters()
{
    return this->pimpl->m_pLegacyGenerator->getDescriptionOfParameters();
}

std::string DynamicsRegressorGenerator::getDescriptionOfParameters(const VectorDynSize& values)
{
    this->pimpl->m_parameters =
        Eigen::Map<const Eigen::VectorXd>(values.data(),
                                          values.size());
    return this->pimpl->m_pLegacyGenerator->getDescriptionOfParameters(this->pimpl->m_parameters);
}

//////////////////////////////////////////////////////////////////////////////
//// Degrees of freedom related methods
//////////////////////////////////////////////////////////////////////////////

unsigned int DynamicsRegressorGenerator::getNrOfDegreesOfFreedom() const
{
    return (unsigned int)this->pimpl->m_pLegacyGenerator->getNrOfDOFs();
}

std::string DynamicsRegressorGenerator::getDescriptionOfDegreeOfFreedom(int dof_index)
{
    return this->pimpl->robot_model.getJunction(dof_index)->getName();
}

std::string DynamicsRegressorGenerator::getDescriptionOfDegreesOfFreedom()
{
    std::stringstream ss;

    for(unsigned int dof = 0; dof < this->getNrOfDegreesOfFreedom(); dof++ )
    {
        ss << "DOF Index: " << dof << " Name: " <<  this->getDescriptionOfDegreeOfFreedom(dof) << std::endl;
    }

    return ss.str();
}

//////////////////////////////////////////////////////////////////////////////
//// Links related methods
//////////////////////////////////////////////////////////////////////////////

/*
unsigned int DynamicsRegressorGenerator::getNrOfLinks() const
{
    assert(false);
    return (unsigned int)this->pimpl->m_pLegacyGenerator->getNrOfDOFs();
}

std::string DynamicsRegressorGenerator::getDescriptionOfLink(int link_index)
{

}

std::string DynamicsRegressorGenerator::getDescriptionOfLinks()
{

}*/



bool DynamicsRegressorGenerator::getModelParameters(VectorDynSize& values)
{
    if( values.size() != this->getNrOfParameters() )
    {
        std::cerr << "[ERROR] error for getModelParameters" << std::endl;
        return false;
    }

    this->pimpl->m_pLegacyGenerator->getModelParameters(this->pimpl->m_parameters);

    Eigen::Map<Eigen::VectorXd>(values.data(),
                                values.size()) =
                                    this->pimpl->m_parameters;

    return true;
}

bool DynamicsRegressorGenerator::setRobotState(const VectorDynSize& q,
                                               const VectorDynSize& q_dot,
                                               const VectorDynSize& q_dotdot,
                                               const Twist& world_gravity)
{
    Transform world_T_base = Transform::Identity();
    Twist base_velocity = SpatialMotionVector::Zero();
    Twist base_acceleration = SpatialMotionVector::Zero();

    return setRobotState(q,q_dot,q_dotdot,
                         world_T_base,base_velocity,base_acceleration,
                         world_gravity);
}

bool DynamicsRegressorGenerator::setRobotState(const VectorDynSize& q,
                                               const VectorDynSize& q_dot,
                                               const VectorDynSize& q_dotdot,
                                               const Transform& world_T_base,
                                               const Twist& base_velocity,
                                               const Twist& base_acceleration,
                                               const Twist& world_gravity)
{
    bool ok = true;
    ok = ok && ToKDL(q,this->pimpl->m_qKDL);
    ok = ok && ToKDL(q_dot,this->pimpl->m_dqKDL);
    ok = ok && ToKDL(q_dotdot,this->pimpl->m_ddqKDL);

    if( !ok )
    {
        std::cerr << "DynamicsRegressorGenerator::setRobotState failed" << std::endl;
        return false;
    }

    // Convert from the new DynamicRegressorGenerator convention to the old one
    Rotation base_R_world = world_T_base.getRotation().inverse();
    Twist base_velocity_wrt_base = base_R_world*base_velocity;
    Twist base_classical_acceleration_wrt_base = base_R_world*base_acceleration;
    Twist gravity_acceleration_wrt_base        = base_R_world*world_gravity;

    KDL::Twist kdl_base_velocity_wrt_base = ToKDL(base_velocity_wrt_base);
    KDL::Twist kdl_classical_base_acceleration = ToKDL(base_classical_acceleration_wrt_base);
    KDL::Twist kdl_spatial_acceleration;

    KDL::CoDyCo::conventionalToSpatialAcceleration(kdl_classical_base_acceleration,kdl_base_velocity_wrt_base,kdl_spatial_acceleration);

    KDL::Twist kdl_spatial_proper_base_acceleration = kdl_spatial_acceleration - ToKDL(gravity_acceleration_wrt_base);

    return (this->pimpl->m_pLegacyGenerator->setRobotState(this->pimpl->m_qKDL,
                                                          this->pimpl->m_dqKDL,
                                                          this->pimpl->m_ddqKDL,
                                                          kdl_base_velocity_wrt_base,
                                                          kdl_spatial_proper_base_acceleration) == 0);

}

SensorsMeasurements& DynamicsRegressorGenerator::getSensorsMeasurements()
{
    return this->pimpl->m_pLegacyGenerator->sensorMeasures;
}


bool DynamicsRegressorGenerator::computeRegressor(MatrixDynSize& regressor, VectorDynSize& known_terms)
{
    if( !this->isValid() )
    {
        return false;
    }

    int ret_value =
        this->pimpl->m_pLegacyGenerator->computeRegressor(this->pimpl->m_regressor,
                                                          this->pimpl->m_knownTerms);

    // \todo TODO write proper header for conversion from/to Eigen
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(regressor.data(),
                                regressor.rows(),
                                regressor.cols()) = this->pimpl->m_regressor;

    Eigen::Map<Eigen::VectorXd>(known_terms.data(),
                                known_terms.size()) = this->pimpl->m_knownTerms;

    return (ret_value == 0);
}

bool DynamicsRegressorGenerator::computeFloatingBaseIdentifiableSubspace(MatrixDynSize& basisMatrix)
{
    if( !this->isValid() )
    {
        return false;
    }

    bool static_regressor = false;
    bool fixed_base = false;
    int ret_value =
        this->pimpl->m_pLegacyGenerator->computeNumericalIdentifiableSubspace(this->pimpl->m_basisMatrix,
                                                                              static_regressor,
                                                                              fixed_base);

    basisMatrix.resize(this->pimpl->m_basisMatrix.rows(),this->pimpl->m_basisMatrix.cols());

    // \todo TODO write proper header for conversion from/to Eigen
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(basisMatrix.data(),
                                basisMatrix.rows(),
                                basisMatrix.cols()) = this->pimpl->m_basisMatrix;

    return (ret_value == 0);
}

bool DynamicsRegressorGenerator::computeFixedBaseIdentifiableSubspace(MatrixDynSize& basisMatrix)
{
    if( !this->isValid() )
    {
        return false;
    }

    bool static_regressor = false;
    bool fixed_base = true;
    int ret_value =
        this->pimpl->m_pLegacyGenerator->computeNumericalIdentifiableSubspace(this->pimpl->m_basisMatrix,
                                                                              static_regressor,
                                                                              fixed_base);

    basisMatrix.resize(this->pimpl->m_basisMatrix.rows(),this->pimpl->m_basisMatrix.cols());

    // \todo TODO write proper header for conversion from/to Eigen
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(basisMatrix.data(),
                                basisMatrix.rows(),
                                basisMatrix.cols()) = this->pimpl->m_basisMatrix;

    return (ret_value == 0);
}



}

}

