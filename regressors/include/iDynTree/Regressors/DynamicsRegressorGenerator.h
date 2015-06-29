/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_DYNREGRESSORGENERATOR_H

#include <string>

namespace iDynTree {

class VectorDynSize;
class MatrixDynSize;
class Transform;
class Twist;
class SensorsList;
class SensorsMeasurements;

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
    struct DynamicsRegressorGeneratorPrivateAttributes;
    DynamicsRegressorGeneratorPrivateAttributes * pimpl;

    // copy is disabled for the moment
    DynamicsRegressorGenerator(const DynamicsRegressorGenerator & other);
    DynamicsRegressorGenerator& operator=(const DynamicsRegressorGenerator& other);

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
    bool loadRobotAndSensorsModelFromFile(const std::string & filename, const std::string & filetype="urdf");

    /**
     * Load the model of the robot and the sensors from a string.
     *
     * @param filename string containg the model of the robot.
     * @param filetype type of the file to load, currently supporting only urdf type.
     *
     */
    bool loadRobotAndSensorsModelFromString(const std::string & modelString, const std::string & filetype="urdf");

    // Coming soon: load model and sensor from iDynTree data structures
    //loadRobotAndSensorsFromModel()

    /**
     * Load the regressor structure from a configuration file.
     *
     * Currently only the subtreeBaseDynamics regressor type is supported.
     * For load the regressor of the subtree that has the `r_arm` as the FTSensorLink,
     * you can pass the following xml string:
     * ~~~
     * <regressor>
     *     <subtreeBaseDynamics>
     *         <FTSensorLink>r_upper_arm</FTSensorLink>
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
     * If you want to exclude some links from the regressor (for example because you know
     *  a-priori that their influence on the dynamics is negligible) you can
     * use the ignoredLink tag:
     *
     * ~~~
     * <regressor>
     *     <subtreeBaseDynamics>
     *         <FTSensorLink>r_upper_arm</FTSensorLink>
     *     </subtreeBaseDynamics>
     *     <ignoredLink>r_elbow_1</ignoredLink>
     *     <ignoredLink>r_wrist_1</ignoredLink>
     * </regressor>
     * ~~~
     *
     * For subtreeBaseDynamics subregressor, you specify the frame of expressions (i.e
     * with respect to which point and to which orientation the regressor equation will be
     * written) using the `<frame>` or `<sensorFrame>` tag.
     *
     * @param filename path to the file to load
     * @return true if the parsing was successful, false otherwise.
     *
     */
    bool loadRegressorStructureFromFile(const std::string & filename);

    /**
     * Load the regressor structure from a configuration string, following the
     * XML structure documented in loadRegressorStructureFromFile .
     *
     * @param regressorStructureString the XML string.
     * @return true if the parsing was successful, false otherwise.
     */
    bool loadRegressorStructureFromString(const std::string & regressorStructureString);

    /**
     * Return true if the models for the robot, sensors and regressors have been correctly
     * loaded and the regressor generator is ready to compute regressors.
     *
     * @return True if the regressor generator is correctly configure, false otherwise.
     */
    bool isValid();
    //@}


    /**
     * @name Methods to get informations about the regressor
     */
    //@{
    /**
    *
    * Get the number of parameters used by the regressor currently generated.
    * This depends on the structure of the regressors added to the regressor generator.
    * The number of the parameters will be the number of columns of the generated
    * regressor.
    *
    * @return the number of parameters used by the regressor currently generated
    */
    unsigned int getNrOfParameters() const;

    /**
     * Get the number of outputs by the regressor currently generated.
     * The number of outputs is the number of rows of the generated regressor.
     *
     * @return the number of outputs of the regressor.
     */
    unsigned int getNrOfOutputs() const;

    /**
     * Get the number of internal degrees of freedom of the robot model used
     * in the regressor generator.
     * This return the *internal* degrees of freedom, because it does not include
     * the eventual 6 degrees of freedom usually associated with the floating base.
     *
     * @return the number of internal degrees of freedom of the regressor.
     */
    unsigned int getNrOfDegreesOfFreedom() const;

    /**
     * Get a human readable description of a given parameter.
     *
     * @param parameter_index the index of the parameter in the parameter vector.
     * @param with_value if true, print the description with also the value provided as an argument.
     * @param value if with_value is true, add this value to the description of the parameter.
     * @return a human readable description of a given parameter.
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
    * Get a human readable description of all the parameters.
    * With respect to getDescriptionOfParameters(), with this version
    * of the method is also possible to print with the description
    * of each parameter its value (this can be useful for printing
    * estimation results).
    *
    * @param values a vector of size getNrOfParameters(), containg the values of the parameters.
    * @return a std::string containg the description of the parameters.
    */
    std::string getDescriptionOfParameters(const iDynTree::VectorDynSize & values);

    /**
     * Get a human readable description of a given output.
     *
     * @return a human readable description of a given output.
     */
    std::string getDescriptionOfOutput(int output_index);

    /**
     * Get a human readable description of all the outputs.
     *
     * @return a human readable description of all the outputs.
     */
    std::string getDescriptionOfOutputs();

    /**
     * Get a human readable description of a given internal degree of freedom of the robot.
     *
     */
    std::string getDescriptionOfDegreeOfFreedom(int dof_index);

    /**
     * Get a human readable description of all the internal degrees of freedom of the robot.
     *
     * @return a std::string containing the description of the internal degrees of freedom.
     */
    std::string getDescriptionOfDegreesOfFreedom();

    /**
     * Get a human readable description of a given link considered in the regressor.
     *
     * @return a human readable description of a given link considered in the regressor.
     */
    //std::string getDescriptionOfLink(int link_index);

    /**
     * Get a human readable description of all links considered in the regressor.
     *
     * @return a std::string containing the description of all the links.
     */
    //std::string getDescriptionOfLinks();

    /**
     * Get name of the base link.
     *
     * @return the name of the base link.
     */
    std::string getBaseLinkName();


    //@}


    /**
     * @name Methods to access the underlyng model for the robot and the sensors
     * (todo)
     */
    //@{

    //const UndirectedTree & getRobotModel();
    const SensorsList & getSensorsModel() const;

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
     * @param q a vector of getNrOfDegreesOfFreedom() joint positions (in rad)
     * @param q_dot a vector of getNrOfDegreesOfFreedom() joint velocities (in rad/sec)
     * @param q_dot a vector of getNrOfDegreesOfFreedom() joint acceleration (in rad/sec^2)
     * @param world_T_base  the homogeneous transformation that transforms position vectors expressed in the base reference frame
     *                      in position frames expressed in the world reference frame (i.e. pos_world = world_T_base*pos_base .
     * @param base_velocity The twist (linear/angular velocity) of the base, expressed in the world orientation frame and with respect
     *                      to the base origin.
     * @param base_acceleration The 6d classical acceleration (linear/angular acceleration) of the base
     *                          expressed in the world orientation frame and with respect to the base origin.
     *                          Plese note that this is *not* the spatial acceleration of the base link.
     *                          For more info check http://wiki.icub.org/codyco/dox/html/dynamics_notation.html .
     *
     * \note this convention is the same used in the wholeBodyInterface classes.
     *
     * \todo TODO define a proper RobotState class, so we centralize all this definitions and we can leave
     *            to the user the freedom of setting spatial/angular acceleration and the reference frames in
     *            which to express quantities.
     */
    bool setRobotState(const iDynTree::VectorDynSize& q,
                       const iDynTree::VectorDynSize& q_dot,
                       const iDynTree::VectorDynSize& q_dotdot,
                       const iDynTree::Transform &world_T_base,
                       const iDynTree::Twist& base_velocity,
                       const iDynTree::Twist& base_acceleration,
                       const iDynTree::Twist& world_gravity);

    /**
     * Set the state for the robot (fixed base)
     * Same as setRobotState, but with:
     *  world_T_base      = iDynTree::Transform::Identity()
     *  base_velocity     = iDynTree::Twist::Zero();
     *  base_acceleration = iDynTree::Twist::Zero();
     *
     * \todo TODO define a RobotState class.
     */
    bool setRobotState(const iDynTree::VectorDynSize &q,
                       const iDynTree::VectorDynSize &q_dot,
                       const iDynTree::VectorDynSize &q_dotdot,
                       const iDynTree::Twist& world_gravity);

    /**
     * Get a reference to the SensorsMeasurements structure, where
     * sensors readings can be set for use of the dynamics regressor generator.
     */
    SensorsMeasurements & getSensorsMeasurements();

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
     * @param[out] values a Vector of size getNrOfParameters() ,
     * @return true if all went well, false if there was an error
     */
    bool getModelParameters(iDynTree::VectorDynSize & values);

    //@}

   /**
    * @name Methods to compute the identifiable subspace of the base parameters
    *
    * \note this methods are implemented here for conveniene, but they are hiding
    *       too many parameters related to the identifiable subspace computations.
    *       They will be probably changed/replaced by something else in the future.
    */
    //@{

     /**
      * Get a matrix whose columns are a basis for the identifiable subspace of
      * the regressor computed by this regressor generator.
      *
      * The algorithm used is the one presented in:
      * Gautier, M. "Numerical calculation of the base inertial parameters of robots."
      *
      * To obtain the so-called "base parameters" (i.e. a projection of the
      *  parameters considered by the regressor on a basis of the identifiable
      *  subspace) you have to multiply the transpose of the returned basisMatrix
      * for the complete vector of the parameters:
      *  baseParameters = basisMatrix.transpose() * completeParameters
      *
      * To obtain the so-called "base regressor" (i.e. the regressor expressed
      *  with respect to the base parameter instead of the complete parameters)
      *  you have to multiply the complete regressor for the returned basisMatrix :
      *  baseRegressor = completeRegressor*basisMatrix
      *
      * \note the basisMatrix will be resize to match the size of the identifiable subspace.
      *
      * @param[out] basisMatrix a Matrix of size getNrOfParameters() X size of identifiable subspace .
      * @return true if all went well, false if there was an error
      */
    bool computeFloatingBaseIdentifiableSubspace(iDynTree::MatrixDynSize & basisMatrix);

    /**
     * Like computeFloatingBaseIdentifiableSubspace, but assuming that the
     * base of the robot is fixed.
     *
     * @param[out] basisMatrix a Matrix of size getNrOfParameters() X size of identifiable subspace .
     * @return true if all went well, false if there was an error
     */
    bool computeFixedBaseIdentifiableSubspace(iDynTree::MatrixDynSize & basisMatrix);


    //@}

private:

};

}

}


#endif
