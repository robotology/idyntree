/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_DYNREGRESSORPARAMETERS_H
#define IDYNTREE_DYNREGRESSORPARAMETERS_H

#include <vector>
#include <string>

namespace iDynTree {
namespace Regressors {
    
/**
 * \ingroup iDynTreeRegressors
 */
enum DynamicsRegressorParameterCategory
{
    LINK_PARAM,
    SENSOR_FT_PARAM
};

/**
 * \ingroup iDynTreeRegressors
 */
enum DynamicsRegressorParameterType
{
    /**
     * Link inertial parameters.
     * For this parameters the DynamicsRegressorParameter::index
     * refer to the link index.
     */
    ///@{
    /**
     * Mass of the link. (Kg)
     */
    LINK_MASS,

    /**
     * Center of mass of the link (expressed in the link frame)
     * multiplied by the mass, X component. (Kg m)
     */
    LINK_FIRST_MOMENT_OF_MASS_X,

    /**
     * Center of mass of the link (expressed in the link frame)
     * multiplied by the mass, Y component. (Kg m)
     */
    LINK_FIRST_MOMENT_OF_MASS_Y,

    /**
     * Center of mass of the link (expressed in the link frame)
     * multiplied by the mass, Z component. (Kg m)
     */
    LINK_FIRST_MOMENT_OF_MASS_Z,

    /**
     * 3D moment of inertia of the link (expressed in the link frame
     * and with respect to the link origin), XX component. (Kg m^2)
     */
    LINK_MOMENT_OF_INERTIA_XX,

    /**
     * 3D moment of inertia of the link (expressed in the link frame
     * and with respect to the link origin), XY component. (Kg m^2)
     */
    LINK_MOMENT_OF_INERTIA_XY,

    /**
     * 3D moment of inertia of the link (expressed in the link frame
     * and with respect to the link origin), XZ component. (Kg m^2)
     */
    LINK_MOMENT_OF_INERTIA_XZ,

    /**
     * 3D moment of inertia of the link (expressed in the link frame
     * and with respect to the link origin), YY component. (Kg m^2)
     */
    LINK_MOMENT_OF_INERTIA_YY,

    /**
     * 3D moment of inertia of the link (expressed in the link frame
     * and with respect to the link origin), YZ component. (Kg m^2)
     */
    LINK_MOMENT_OF_INERTIA_YZ,

    /**
     * 3D moment of inertia of the link (expressed in the link frame
     * and with respect to the link origin), ZZ component. (Kg m^2)
     */
    LINK_MOMENT_OF_INERTIA_ZZ,

    ///@}


    /**
     * Six Axis FT sensor parameters.
     * For this parameters the DynamicsRegressorParameter::index
     * refer to the FT sensor index.
     *
     * The offset o is defined in a way such that, if r is the raw
     * output of the sensor and f is the actual wrench excerted throught
     * the sensor, we have:
     * r = f + o
     *
     */
    ///@{
    /**
     * Sensor offset for a six axis FT sensor, expressed in the sensor
     * frame. X component of the force offset.
     */
    SENSOR_FT_OFFSET_FORCE_X,

    /**
     * Sensor offset for a six axis FT sensor, expressed in the sensor
     * frame. Y component of the force offset.
     */
    SENSOR_FT_OFFSET_FORCE_Y,

    /**
     * Sensor offset for a six axis FT sensor, expressed in the sensor
     * frame. Z component of the force offset.
     */
    SENSOR_FT_OFFSET_FORCE_Z,

    /**
     * Sensor offset for a six axis FT sensor, expressed in the sensor
     * frame. X component of the torque offset.
     */
    SENSOR_FT_OFFSET_TORQUE_X,

    /**
     * Sensor offset for a six axis FT sensor, expressed in the sensor
     * frame. Y component of the torque offset.
     */
    SENSOR_FT_OFFSET_TORQUE_Y,

    /**
     * Sensor offset for a six axis FT sensor, expressed in the sensor
     * frame. Z component of the torque offset.
     */
    SENSOR_FT_OFFSET_TORQUE_Z
};

/**
 * \ingroup iDynTreeRegressors
 */
struct DynamicsRegressorParameter
{
    DynamicsRegressorParameterCategory category;

    /**
     * Depending on the category, the parameter will referent
     * to an element (such as a link or a sensor) throught an
     * integer. This integer should be interpreted with respect
     * to the serialization provided by the SensorList class
     * or the UndirectedTree class.
     */
    int elemIndex;
    DynamicsRegressorParameterType type;

    bool operator<(const DynamicsRegressorParameter& other) const;
    bool operator==(const DynamicsRegressorParameter& other) const;
    bool operator!=(const DynamicsRegressorParameter& other) const;
};

/**
 * \ingroup iDynTreeRegressors
 */
class DynamicsRegressorParametersList
{
public:
    // \todo TODO pimpl and hide
    // this vector is kept orderd by the addList and addParam
    // methods, do not insert element manually
    std::vector<DynamicsRegressorParameter> parameters;

    std::string getDescriptionOfParameter(unsigned int param_index) const;

    /**
     * Temporary function, until we add a SensorList/Tree aware
     * DynamicsRegressorParametersList . We provide a string to
     * substitute the print of the elemIndex element of the DynamicsRegressorParameter
     */
    std::string getDescriptionOfParameter(unsigned int param_index, const std::string elemName) const;


    /**
     * Add a new parameter to the list.
     *
     */
    bool addParam(const DynamicsRegressorParameter & param);

    /**
     * Add a list of parameters to this list.
     *
     * @param list the list of parameters to add.
     * @return true if all went well, false otherwise.
     */
    bool addList(const DynamicsRegressorParametersList & new_params);

    /**
     * Search for a param in the list.
     * If found, store the index in the output parameter findIndex.
     */
    bool findParam(const DynamicsRegressorParameter & param, unsigned int & index) const;

    /**
     * Get the number of parameters in this list.
     *
     * @return the number of parameters contained in this list in this list.
     */
    unsigned int getNrOfParameters() const;
};

}
}

#endif
