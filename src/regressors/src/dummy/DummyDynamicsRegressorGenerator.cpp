/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iDynTree/Regressors/DynamicsRegressorGenerator.h"

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Rotation.h>

#include <iDynTree/Sensors/Sensors.h>

namespace iDynTree
{

namespace Regressors
{

struct DynamicsRegressorGenerator::DynamicsRegressorGeneratorPrivateAttributes
{
    SensorsList dummySensorsList;
    SensorsMeasurements dummySensorsMeasurements;
};

void printDynamicsRegressorGeneratorDeprecation()
{
    std::cerr << "[WARNING] The iDynTree::Regressors::DynamicsRegressorGenerator class still needs to be migrated to" << std::endl <<
                 "[WARNING] iDynTree native classes. To continue to use the iDynTree::HighLevel::DynamicsComputations class in the " << std::endl <<
                 "[WARNING] bindings in the current form, please compile iDynTree with the IDYNTREE_USES_KDL CMake option."  << std::endl;
}

DynamicsRegressorGenerator::DynamicsRegressorGenerator():
pimpl(new DynamicsRegressorGeneratorPrivateAttributes)
{
    printDynamicsRegressorGeneratorDeprecation();
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
    delete this->pimpl;
    this->pimpl = 0;
}

bool DynamicsRegressorGenerator::loadRobotAndSensorsModelFromFile(const std::string&,
                                                                  const std::string&)
{
    printDynamicsRegressorGeneratorDeprecation();
    return false;
}

bool DynamicsRegressorGenerator::loadRobotAndSensorsModelFromString(const std::string&,
                                                                    const std::string&)
{
    printDynamicsRegressorGeneratorDeprecation();
    return false;
}


bool DynamicsRegressorGenerator::loadRegressorStructureFromFile(const std::string&)
{
    printDynamicsRegressorGeneratorDeprecation();
    return false;
}

bool DynamicsRegressorGenerator::loadRegressorStructureFromString(const std::string& regressorStructureString)
{
    printDynamicsRegressorGeneratorDeprecation();
    return false;
}


bool DynamicsRegressorGenerator::isValid()
{
    printDynamicsRegressorGeneratorDeprecation();
    return false;
}

const SensorsList& DynamicsRegressorGenerator::getSensorsModel() const
{
    printDynamicsRegressorGeneratorDeprecation();
    return this->pimpl->dummySensorsList;
}

std::string DynamicsRegressorGenerator::getBaseLinkName()
{
    printDynamicsRegressorGeneratorDeprecation();
    return "";
}

//////////////////////////////////////////////////////////////////////////////
//// Output related methods
//////////////////////////////////////////////////////////////////////////////

unsigned int DynamicsRegressorGenerator::getNrOfOutputs() const
{
    printDynamicsRegressorGeneratorDeprecation();
    return 0;
}

std::string DynamicsRegressorGenerator::getDescriptionOfOutput(int)
{
    printDynamicsRegressorGeneratorDeprecation();
    return "";
}

std::string DynamicsRegressorGenerator::getDescriptionOfOutputs()
{
    printDynamicsRegressorGeneratorDeprecation();
    return "";
}

//////////////////////////////////////////////////////////////////////////////
//// Parameters related methods
//////////////////////////////////////////////////////////////////////////////

unsigned int DynamicsRegressorGenerator::getNrOfParameters() const
{
    printDynamicsRegressorGeneratorDeprecation();
    return 0;
}

std::string DynamicsRegressorGenerator::getDescriptionOfParameter(int parameter_index, bool with_value, double value)
{
    printDynamicsRegressorGeneratorDeprecation();
    return "";
}

std::string DynamicsRegressorGenerator::getDescriptionOfParameters()
{
    printDynamicsRegressorGeneratorDeprecation();
    return "";
}

std::string DynamicsRegressorGenerator::getDescriptionOfParameters(const VectorDynSize& values)
{
    printDynamicsRegressorGeneratorDeprecation();
    return "";
}

//////////////////////////////////////////////////////////////////////////////
//// Degrees of freedom related methods
//////////////////////////////////////////////////////////////////////////////

unsigned int DynamicsRegressorGenerator::getNrOfDegreesOfFreedom() const
{
    printDynamicsRegressorGeneratorDeprecation();
    return 0;
}

std::string DynamicsRegressorGenerator::getDescriptionOfDegreeOfFreedom(int dof_index)
{
    printDynamicsRegressorGeneratorDeprecation();
    return "";
}

std::string DynamicsRegressorGenerator::getDescriptionOfDegreesOfFreedom()
{
    printDynamicsRegressorGeneratorDeprecation();
    return "";
}

//////////////////////////////////////////////////////////////////////////////
//// Links related methods
//////////////////////////////////////////////////////////////////////////////


unsigned int DynamicsRegressorGenerator::getNrOfLinks() const
{
    printDynamicsRegressorGeneratorDeprecation();
    return 0;
}

unsigned int DynamicsRegressorGenerator::getNrOfFakeLinks() const
{
    printDynamicsRegressorGeneratorDeprecation();
    return 0;
}


std::string DynamicsRegressorGenerator::getDescriptionOfLink(int link_index)
{
    printDynamicsRegressorGeneratorDeprecation();
    return "";
}

std::string DynamicsRegressorGenerator::getDescriptionOfLinks()
{
    printDynamicsRegressorGeneratorDeprecation();
    return "";
}


bool DynamicsRegressorGenerator::getModelParameters(VectorDynSize& values)
{
    printDynamicsRegressorGeneratorDeprecation();
    return false;
}

bool DynamicsRegressorGenerator::setRobotState(const VectorDynSize& q,
                                               const VectorDynSize& q_dot,
                                               const VectorDynSize& q_dotdot,
                                               const Twist& world_gravity)
{
    printDynamicsRegressorGeneratorDeprecation();
    return false;
}

bool DynamicsRegressorGenerator::setRobotState(const VectorDynSize& q,
                                               const VectorDynSize& q_dot,
                                               const VectorDynSize& q_dotdot,
                                               const Transform& world_T_base,
                                               const Twist& base_velocity,
                                               const Twist& base_acceleration,
                                               const Twist& world_gravity)
{
    printDynamicsRegressorGeneratorDeprecation();
    return false;
}

SensorsMeasurements& DynamicsRegressorGenerator::getSensorsMeasurements()
{
    printDynamicsRegressorGeneratorDeprecation();
    return this->pimpl->dummySensorsMeasurements;
}

int DynamicsRegressorGenerator::setTorqueSensorMeasurement(const int, const double)
{
    printDynamicsRegressorGeneratorDeprecation();
    return -1;
}


int DynamicsRegressorGenerator::setTorqueSensorMeasurement(iDynTree::VectorDynSize &)
{
    printDynamicsRegressorGeneratorDeprecation();
    return -1;
}

bool DynamicsRegressorGenerator::computeRegressor(MatrixDynSize& , VectorDynSize& )
{
    printDynamicsRegressorGeneratorDeprecation();
    return false;
}

bool DynamicsRegressorGenerator::computeFloatingBaseIdentifiableSubspace(MatrixDynSize&)
{
    printDynamicsRegressorGeneratorDeprecation();
    return false;
}

bool DynamicsRegressorGenerator::computeFixedBaseIdentifiableSubspace(MatrixDynSize&)
{
    printDynamicsRegressorGeneratorDeprecation();
    return false;
}

int DynamicsRegressorGenerator::generate_random_regressors(iDynTree::MatrixDynSize & output_matrix, const bool static_regressor,
                                   const bool fixed_base,
                                   int n_samples)
{
    printDynamicsRegressorGeneratorDeprecation();
    return -1;
}


}

}

