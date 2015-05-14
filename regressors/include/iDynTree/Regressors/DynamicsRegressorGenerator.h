/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_DYNREGRESSORGENERATOR_H

namespace iDynTree {

class VectorDynSize;
class MatrixDynSize;
class Transform;
class Twist;

namespace Regressors {

/**
 * The dynamics regressor generator is a class for calculating arbitrary regressor
 * related to robot dynamics, for identification of dynamics parameters, such
 * as inertial parameters (masses, centers of mass, inertia tensor elements) or
 * other related parameters (for example force/torque sensor offset).
 *
 */
class DynamicsRegressorGenerator {
private:
    struct DynamicsRegressorGeneratorPimpl;
    DynamicsRegressorGeneratorPimpl * pimpl;

public:

    /**
     * @name Constructor/Destructor
     */
    //@{

    /**
     * Constructor
     *
     */
    DynamicsRegressorGenerator();

    /**
     * Destructor
     *
     */
    virtual ~DynamicsRegressorGenerator();
    //@}

    /**
     * @name Model loading and definition methods
     * This methods are used to define the structure of your regressor, i.e. load the model
     * and define the desired regressors.
     */
    //@{

    /**
     * Load the model of the robot and the sensors from an external file.
     *
     * @param filename path to the file to load
     * @param filetype type of the file to load, currently supporting only urdf type.
     *
     */
    bool loadRobotAndSensorsModelFromFile(const std::string & filename, const std::string & filetype="detect");

    /**
     * Load the model of the robot and the sensors from a string.
     *
     * @param filename string containg the model of the robot.
     * @param filetype type of the file to load, currently supporting only urdf type.
     *
     */
    bool loadRobotAndSensorsModelFromString(const std::string & modelString, const std::string & filetype="detect");

    // Coming soon: load model and sensor from iDynTree data structures
    //loadRobotAndSensorsModel()

    /**
     * Load the regressor structure from a configuration file.
     *
     * Currently only the subtreeBaseDynamics regressor type is supported.
     * For load the regressor of the subtree that has the `r_arm` as the FTSensorLink,
     * you can pass the following xml string:
     * ~~~
     * <regressor>
     *     <subtreeBaseDynamics>
     *         <FTSensorLink>r_arm</FTSensorLink>
     *     </subtreeBaseDynamics>
     * </regressor>
     * ~~~
     *
     * For loading the regressor composed of the two subtrees, one that has the `l_foot` as FTSensorLink,
     * and the other that has `l_ankle_2` and `l_hip_3` has FTSensorLink, you can pass the following xml string:
     *
     * ~~~
     * <regressor>
     *     <subtreeBaseDynamics>
     *         <FTSensorLink>l_foot</FTSensorLink>
     *     </subtreeBaseDynamics>
     *     <subtreeBaseDynamics>
     *         <FTSensorLink>l_ankle_2</FTSensorLink>
     *         <FTSensorLink>l_hip_3</FTSensorLink>
     *     </subtreeBaseDynamics>
     * </regressor>
     * ~~~
     *
     * For subtreeBaseDynamics subregressor, you specify the frame of expressions (i.e
     * with respect to which point and to which orientation the regressor equation will be
     * written) using the `<frame>` or `<sensorFrame>` tag.
     *
     * @param filename path to the file to load
     * @param filetype type of the file to load, currently supporting only urdf type.
     */
    loadRegressorStructureFromFile(const std::string & filename);

    loadRegressorStructureFromString(const std::string & regressorStructureString);

    /**
     * Configure the regressor generator, i.e. check that the model and
     * the regressor structure is compatible.
     */
    bool configure();

    /**
     * Return true if the configure call was successfull.
     */
    bool isValid();
    //@}


    /**
     * @name Methods to get informations about the regressor
     */
    //@{
    /**
    *
    * Get the number of parameters used by the regressor currently generated (i.e. the number of rows of the regressor)
    * (default: 10*tree.getNrOfSegments(), then depending on the type of regressor more can be added)
    *
    */
    int getNrOfParameters() const;

    /**
     * Get the number of outputs by the regressor currently generated (i.e. the number of columns of the regressor)
     *
     */
    int getNrOfOutputs() const;

    /**
     * Get a human readable description of a given parameter
     */
    std::string getDescriptionOfParameter(int parameter_index,
                                          bool with_value=false,
                                          double value=-1.0);

    /**
     * Get a human readable description of all the parameters
     *
     */
    std::string getDescriptionOfParameters();


    /**
    * Get a human readable description of all the parameters
    *
    * @return a std::string containg the description of the parameters
    */
    std::string getDescriptionOfParameters(const iDynTree::VectorDynSize & values);

    /**
     * Get a human readable description of a given output
     *
     */
    std::string getDescriptionOfOutput(int output_index);

    /**
     * Get a human readable description of all the outputs
     *
     */
    std::string getDescriptionOfOutputs();

    //@}


    /**
     * @name Methods to access the underlyng model for the robot and the sensors
     *
     */
    //@{

    //const UndirectedTree & getRobotModel();
    //const SensorsTree    & getSensorsModel();

    //@}

    /**
      * @name Methods to submit the input data for calculating the regressor and the known terms
      * This methods are used to submit the input data (such as the robot state and the sensor readings)
      * to calculate the regressor and the known terms
      */
    //@{

    /**
     * Set the state for the robot (floating base)
     *
     */
    bool setRobotState(const iDynTree::VectorDynSize& q,
                       const iDynTree::VectorDynSize& q_dot,
                       const iDynTree::VectorDynSize& q_dotdot,
                       const iDynTree::Transform &world_T_base,
                       const iDynTree::Twist& base_velocity,
                       const iDynTree::Twist& base_acceleration
                       const iDynTree::Twist& world_gravity);

    /**
     * Set the state for the robot (fixed base)
     * Same as setRobotState, but with:
     *  world_T_base      = iDynTree::Transform::Identity()
     *  base_velocity     = iDynTree::Twist::Zero();
     *  base_acceleration = iDynTree::Twist::Zero();
     */
    bool setRobotState(const iDynTree::VectorDynSize &q,
                       const iDynTree::VectorDynSize &q_dot,
                       const iDynTree::VectorDynSize &q_dotdot,
                       const iDynTree::Twist& world_gravity);

    /**
     * Get a reference to the SensorsMeasurements structure, where
     * sensors readings can be set for use of the dynamics regressor generator.
     */
    //SensorsMeasurements & getSensorsMeasurements();

    //@}

    /** @name Methods to calculate the regressor
       */
    //@{

    /**
     * Return the compute regressor (computed from the robot state) and the corresponding output (calculated from sensor readings)
     * @param regressor a getNrOfOutputs() times getNrOfUnknownParameters() Matrix
     * @param known_terms a getNrOfOutputs() parameters Vector
     * @return true if all went well, false if there was an error
     */
    bool computeRegressor(iDynTree::MatrixDynSize & regressor,
                          iDynTree::VectorDynSize & known_terms);

    /**
     * Return the dynamics parameters related to this regressor contained in the model.
     *
     * @param[out] a getNrOfUnknownParameters() Vector,
     * @return true if all went well, false if there was an error
     */
    bool getModelParameters(iDynTree::VectorDynSize & values);

    //@}

private:

};

}

}


#endif
