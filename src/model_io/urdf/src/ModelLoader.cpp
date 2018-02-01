/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <string>
#include <vector>

#include <iDynTree/Sensors/ModelSensorsTransformers.h>

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/ModelIO/URDFModelImport.h>
#include <iDynTree/ModelIO/URDFGenericSensorsImport.h>

namespace iDynTree
{

ModelLoader::ModelLoader(): m_isModelValid(false)
{
}

const Model& ModelLoader::model()
{
    return m_model;
}

const SensorsList& ModelLoader::sensors()
{
    return m_sensors;
}

bool ModelLoader::isValid()
{
    return m_isModelValid;
}

bool ModelLoader::setModelAndSensors(const Model& _model,
                                     const SensorsList& _sensors)
{

    m_model = _model;
    m_sensors = _sensors;

    // TODO \todo add a self consistency check of model/sensors
    m_isModelValid = true;

    if( !m_isModelValid )
    {
        reportError("ModelLoader","setModelAndSensors","Loading failed, resetting ModelLoader to be invalid");
        m_model = Model();
        m_sensors = SensorsList();
    }

    return m_isModelValid;
}

bool ModelLoader::loadModelFromFile(const std::string& filename,
                                    const std::string& /*filetype*/)
{
    Model _model;
    SensorsList _sensors;

    bool parsingCorrect = false;

    parsingCorrect = modelFromURDF(filename,_model);

    if( !parsingCorrect )
    {
        reportError("ModelLoader","loadModelFromFile","Error in parsing model from URDF.");
        return false;
    }

    parsingCorrect = sensorsFromURDF(filename,_model,_sensors);

    if( !parsingCorrect )
    {
        reportError("ModelLoader","loadModelFromFile","Error in parsing sensors from URDF.");
        return false;
    }

    return setModelAndSensors(_model,_sensors);
}

bool ModelLoader::loadModelFromString(const std::string& modelString,
                                      const std::string& /*filetype*/)
{
    Model _model;
    SensorsList _sensors;

    bool parsingCorrect = false;

    parsingCorrect = modelFromURDFString(modelString,_model);

    if( !parsingCorrect )
    {
        reportError("loadModelFromString","loadModelFromString","Error in parsing model from URDF.");
        return false;
    }

    parsingCorrect = sensorsFromURDFString(modelString,_model,_sensors);

    if( !parsingCorrect )
    {
        reportError("loadModelFromString","loadModelFromString","Error in parsing sensors from URDF.");
        return false;
    }

    return setModelAndSensors(_model,_sensors);
}

bool ModelLoader::loadReducedModelFromFullModel(const Model& fullModel,
                                                const std::vector< std::string >& consideredJoints,
                                                const std::string /*filetype*/)
{
    SensorsList _sensorsFull, _sensorsReduced;
    Model _modelReduced;
    bool ok = createReducedModelAndSensors(fullModel,_sensorsFull,consideredJoints,_modelReduced,_sensorsReduced);

    if( !ok )
    {
        return false;
    }

    return setModelAndSensors(_modelReduced,_sensorsReduced);
}

bool ModelLoader::loadReducedModelFromString(const std::string modelString,
                                             const std::vector< std::string >& consideredJoints,
                                             const std::string /*filetype*/)
{
    SensorsList _sensorsFull, _sensorsReduced;
    Model _modelFull, _modelReduced;

    bool parsingCorrect = modelFromURDFString(modelString,_modelFull);

    if( !parsingCorrect )
    {
        reportError("ModelLoader","loadReducedModelFromString","Error in parsing model from URDF.");
        return false;
    }

    parsingCorrect = sensorsFromURDFString(modelString,_modelFull,_sensorsFull);

    if( !parsingCorrect )
    {
        reportError("ModelLoader","loadReducedModelFromString","Error in parsing sensors from URDF.");
        return false;
    }

    parsingCorrect = createReducedModelAndSensors(_modelFull,_sensorsFull,consideredJoints,_modelReduced,_sensorsReduced);

    if( !parsingCorrect )
    {
        return false;
    }

    return setModelAndSensors(_modelReduced,_sensorsReduced);
}

bool ModelLoader::loadReducedModelFromFile(const std::string filename,
                                           const std::vector< std::string >& consideredJoints,
                                           const std::string /*filetype*/)
{
    SensorsList _sensorsFull, _sensorsReduced;
    Model _modelFull, _modelReduced;

    bool parsingCorrect = modelFromURDF(filename,_modelFull);

    if( !parsingCorrect )
    {
        reportError("ModelLoader","loadReducedModelFromFile","Error in parsing model from URDF.");
        return false;
    }

    parsingCorrect = sensorsFromURDF(filename,_modelFull,_sensorsFull);

    if( !parsingCorrect )
    {
        reportError("ModelLoader","loadReducedModelFromFile","Error in parsing sensors from URDF.");
        return false;
    }

    parsingCorrect = createReducedModelAndSensors(_modelFull,_sensorsFull,consideredJoints,_modelReduced,_sensorsReduced);

    if( !parsingCorrect )
    {
        return false;
    }

    return setModelAndSensors(_modelReduced,_sensorsReduced);
}


}

