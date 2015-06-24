/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef KDL_CODYCO_DYNREGRESSORGENERATOR_HPP
#define KDL_CODYCO_DYNREGRESSORGENERATOR_HPP

#include <kdl/tree.hpp>

#include "kdl_codyco/utils.hpp"
#include "kdl_codyco/undirectedtree.hpp"
#include "iDynTree/Sensors/Sensors.hpp"
#include "iDynTree/Core/Wrench.h"

//Type of regressors
#include "dynamicRegressorInterface.hpp"
#include "subtreeBaseDynamicsRegressor.hpp"
#include "torqueRegressor.hpp"
#include "baseDynamicsRegressor.hpp"

#include "DynamicSample.hpp"

namespace KDL {
namespace CoDyCo {

namespace Regressors {

/**
 * The dynamics regressor generator is a class for calculating arbitrary regressor
 * related to robot dynamics, for identification of dynamics parameters, such
 * as inertial parameters (masses, centers of mass, inertia tensor elements) or
 * other related parameters (for example force/torque sensor offset).
 *
 *
 */
class DynamicRegressorGenerator {

public:

     /** @name Constructor/Destructor
       */
    //@{

    /**
    * Constructor for DynamicRegressorGenerator
    *
    * \note the dynamics base, used to define forward/backward direction of torque regressor, is
    *       the root of the KDL::Tree
    *
    * @param[in] undirected_tree the KDL::CoDyCo::UndirectedTree description of the robot structure
    * @param[in] sensors_tree    the KDL::CoDyCo::SensorsTree description of the sensors mounted on the robot
    * @param kinematic_base (optional) the link used as the root for propagation of kinematic information (default: the base of the KDL::Tree )
    * @param ft_sensor_names (optional) the names of the Joints (of type Joint::None) that are Force Torque sensors (default: no sensors)
    * @param ft_sensor_offset (optional) consider the ft offset as parameters while building the regressor and the known terms (default: true)
    * @param fake_link_names (optional) a list of names of links to consider without inertia (i.e. with all zero inertial parameters) (default: no links)
    */
    DynamicRegressorGenerator(const KDL::CoDyCo::UndirectedTree & undirected_tree,
                              const iDynTree::SensorsList    & sensors_tree,
                              std::string kinematic_base="",
                              bool ft_sensor_offset=true,
                              std::vector< std::string > fake_link_names=std::vector< std::string >(0),
                              const bool _verbose=false
                             );

    ~DynamicRegressorGenerator() { };
    //@}
    int changeDynamicBase(std::string new_dynamic_base_name);

    int changeKinematicBase(std::string new_kinematic_base_name);

    int getDynamicBaseIndex();
    int getKinematicBaseIndex();


     /** @name Methods to add rows to the regressor
       *  This methods are used to build the structure of your regressor, adding rows to it.
       */
    //@{

    /**
     * Add the regressor relative to a set of internal 6 axis force/torque sensors.
     *
     *
     */
    int addSubtreeRegressorRows(const std::vector< std::string>& _subtree_leaf_links);

    /**
     * Add to the regressor the row relative to a given torque sensor
     *
     */
    int addTorqueRegressorRows(const std::string & dof_name, const bool reverse_direction = false, const std::vector<bool> &_activated_ft_sensors=std::vector<bool>(0));

    /**
     * Add to the regressor the row relative to a given torque sensor
     *
     */
    int addTorqueRegressorRows(const std::string & dof_name, const bool reverse_direction, const std::vector<std::string> &_activated_ft_sensors);

    /**
     * Add to the regressor the rows relative to all the torque of the (internal) degrees of freedom of the robot
     *
     */
    int addAllTorqueRegressorRows();

    /**
     * Add the base dynamics regressor (i.e. the force/torque plate regressor).
     * The regressor output wrench reference frame will be coincident to the base link.
     *
     * \note for this regressor no known terms are automatically calculated
     *
     * \todo add a proper force/plate sensor
     */
    int addBaseRegressorRows();


    /**
     * Call configure after adding all the rows that you want to add to the regressor,
     * to finalize the regressor structure.
     *
     * @return true if the configuration was successful, false otherwise.
     */
    bool configure();

    //@}

     /** @name Methods to get informations the regressor
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
     * Get the number of degrees of freedom of the considered structure
     *
     */
    int getNrOfDOFs() const;

    /**
     * Get the number of wrench (force/torque) sensor of the regressor
     *
     */
    int getNrOfWrenchSensors() const;

    //The feature of fixing/unfixing parameters would be implemented in a later version
    //int getNrOfUnknownParameters();

    //int getNrOfFixedParameters();

    //fixParameter(ParameterIndex par_index, double parameter_value);
    //fixParameter(FixedParameterIndex fixed_par_index, double
    //double getFixedParameter(FixedParameterIndex fixed_par_index);
    //unfixParameter(ParameterIndex par_index);


    /**
     * Get a human readable description of a given parameter
     */
    std::string getDescriptionOfParameter(int parameter_index,bool with_value=false,double value=-1.0);

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
    std::string getDescriptionOfParameters(const Eigen::VectorXd & values);

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

    //std::string getDescriptionOfFixedParameters();

    //std::string getDescriptionOfUnknownParameters();

    /**
      * @name Methods to submit the input data for calculating the regressor and the known terms
      * This methods are used to submit the input data (such as the robot state and the sensor readings) to calculate the regressor and the known terms
      */
    //@{
    /**
     * Set the state for the robot (floating base)
     *
     */
    int setRobotState(const KDL::JntArray &q,
                      const KDL::JntArray &q_dot,
                      const KDL::JntArray &q_dotdot,
                      const KDL::Twist& base_velocity,
                      const KDL::Twist& base_acceleration);

    /**
     * Set the state for the robot (fixed base)
     *
     */
    int setRobotState(const KDL::JntArray &q,
                      const KDL::JntArray &q_dot,
                      const KDL::JntArray &q_dotdot,
                      const KDL::Twist& base_gravity);

    /**
     * Set the state of the robot and the sensor measures
     *
     * @param sample the dirl::DynamicSample object containg the information about the robot state and
     *               the sensor samples
     */
    int setRobotStateAndSensors(const DynamicSample & sample);

    //Eventually the call would be only setSensor and the sensor type
    //subsystem would be more flexible, for now we do in this way

    /**
     * Set the measurement for a given Force/Torque sensor
     *
     * @param ft_sensor_index the index of the FT sensor, an integer from 0 to NrOfFTSensors-1
     */
    int setFTSensorMeasurement(const int ft_sensor_index, const iDynTree::Wrench ftm);

    /**
     * Set the measurement for a given torque sensor
     *
     * @param dof_index the index of the degree of freedom on which the torque was measured, an interger from 0 to NrOfDOFs-1
     */
    int setTorqueSensorMeasurement(const int dof_index,const double measure);

    /**
     * Set the measurements for all (internal degrees of freedom torques
     *
     */
    int setTorqueSensorMeasurement(const KDL::JntArray & torques);
    //@}

    /** @name Method to calculate the regressor
       */
    //@{
    /**
    * Return the compute regressor and the corresponding output (calculate from sensor readings)
    * @param regressor a getNrOfOutputs() times getNrOfUnknownParameters() Matrix
    * @param known_terms a getNrOfOutputs() parameters Vector
    */
    int computeRegressor(Eigen::MatrixXd & regressor, Eigen::VectorXd& known_terms);
    //@}

    /** @name Methods used to obtain the base parameters for the regressor
       */
    //@{
    /**
     * Get a matrix whose columns are the base parameters of the given regressor,
     * i.e. a matrix whose columns are a basis for the identifiable subspace of the given regressor
     *
     * The algorithm used is the one presented in:
     * Gautier, M. "Numerical calculation of the base inertial parameters of robots."
     *
     * @param basis the orthogonal matrix whose columns are a basis of the identifiable subspace
     * @param static_regressor if true, compute the identifiable parameter considering only static poses (default: false)
     * @param fixed_base if true, consider the kinematic base as fixed (i.e. do not vary the position and/or the velocity/acceleration of the kinematic base)
     * @param grav_direction in case of fixed base, get the gravity vector for the fixed kinematic base
     * @param tol the tollerance used in troncating the svd (defaul: max(sigma)*machine_eps*number_of_generated_samples)
     * @param n_samples
     * @param verbose
     *
     * \note This method is not real time safe.
     */
    int computeNumericalIdentifiableSubspace(Eigen::MatrixXd & basis,
                                             const bool static_regressor = false,
                                             const bool fixed_base = false,
                                             const KDL::Vector grav_direction=KDL::Vector(0.0,0.0,9.8),
                                             const std::vector<int> fixed_dofs=std::vector<int>(0),
                                             const std::vector<double> fixed_dofs_values=std::vector<double>(0),
                                             double tol = -1.0,
                                             int n_samples = 1000,
                                             const bool verbose = false);

    /**
     * Get a matrix whose columns are the base parameters of the given regressor,
     * i.e. a matrix whose columns are a basis for the identifiable subspace of the given regressor
     *
     * The algorithm used is the one presented in:
     * Gautier, M. "Numerical calculation of the base inertial parameters of robots."
     *
     * @param basis the orthogonal matrix whose columns are a basis of the identifiable subspace
     * @param static_regressor if true, compute the identifiable parameter considering only static poses (default: false)
     * @param n_samples the number of random regressors to calculate to estimate the identifable subspace
     * @param tol the tollerance used in troncating the svd (defaul: max(sigma)*machine_eps*number_of_generated_samples)
     *
     * \note This method is not real time safe.
     */
    int computeNumericalIdentifiableSubspace(Eigen::MatrixXd & basis, const bool static_regressor, int n_samples = 1000,  double tol = -1.0, const bool verbose = false);


    std::string analyseBaseSubspace(const Eigen::MatrixXd & basis, int verbosity_level = 0, double tol = -1.0);

    std::string analyseSparseBaseSubspace(const Eigen::MatrixXd & basis, double tol=-1.0, bool only_summary = true);


    /**
     * Under testing
     *
     */
    int computeSparseNumericalIdentifiableSubspaceV1( Eigen::MatrixXd & basis, const bool static_regressor = false, const bool fixed_base = false, const KDL::Vector grav_direction=KDL::Vector(0.0,0.0,9.8), double tol = -1.0, int n_samples = 50, const bool verbose = false);

    int computeSparseNumericalIdentifiableSubspaceV2( Eigen::MatrixXd & basis, const bool static_regressor = false, const bool fixed_base = false, const KDL::Vector grav_direction=KDL::Vector(0.0,0.0,9.8), double tol = -1.0, int n_samples = 1000, const bool verbose = false);


    int computeSparseNumericalIdentifiableSubspaceSimpleAlgorithm( Eigen::MatrixXd & basis, const bool static_regressor = false, const bool fixed_base = false, const KDL::Vector grav_direction=KDL::Vector(0.0,0.0,9.8), double tol = -1.0, int n_samples = 100, const bool verbose = false);

    int computeSparseNumericalIdentifiableSubspaceAdvancedAlgorithm( Eigen::MatrixXd & basis, const bool static_regressor = false, const bool fixed_base = false, const KDL::Vector grav_direction=KDL::Vector(0.0,0.0,9.8), double tol = -1.0, int n_samples = 100, const bool verbose = false);

    int computeSparseNumericalIdentifiableSubspaceSimplePaper( Eigen::MatrixXd & basis, const bool static_regressor = false, const bool fixed_base = false, const KDL::Vector grav_direction=KDL::Vector(0.0,0.0,9.8), double tol = -1.0, int n_samples = 100, const bool verbose = false);

    int computeSparseNumericalIdentifiableSubspaceSimpleGolub( Eigen::MatrixXd & basis, const bool static_regressor = false, const bool fixed_base = false, const KDL::Vector grav_direction=KDL::Vector(0.0,0.0,9.8), double tol = -1.0, int n_samples = 100, const bool verbose = false);

    int computeSparseNumericalIdentifiableSubspaceAdvancedPaper( Eigen::MatrixXd & basis, const bool static_regressor = false, const bool fixed_base = false, const KDL::Vector grav_direction=KDL::Vector(0.0,0.0,9.8), double tol = -1.0, int n_samples = 100, const bool verbose = false);

    /**
     * Get an updated Undirected Tree model with the parameters specified in input
     * @return 0 if all went well, -1 otherwise
     */
    int getUpdatedModel(const Eigen::VectorXd & values, KDL::CoDyCo::UndirectedTree & updated_model);

    int getModelParameters(Eigen::VectorXd & values);

    iDynTree::SensorsList sensorsList;

    //measured 6 axis Force/torques
    iDynTree::SensorsMeasurements sensorMeasures;

private:
    KDL::CoDyCo::UndirectedTree undirected_tree; /**< UndirectedTree object: it encodes the TreeSerialization */

    KDL::CoDyCo::Traversal dynamic_traversal;
    KDL::CoDyCo::Traversal kinematic_traversal; /**< Traversal object: defining the kinematic base of the tree */

    // Vector containing the desciption of the parameters used in this regressor
    iDynTree::Regressors::DynamicsRegressorParametersList parameters_desc;

    //Violating DRY principle, but for code clarity
    int NrOfFakeLinks;
    int NrOfDOFs;
    int NrOfRealLinks_gen;
    int NrOfAllPossibleParameters; /**< Define the maximum number of parameters */
    int NrOfOutputs;

    //Take in account the real and fake links
    std::vector< bool > is_link_real;
    std::vector< int > regrColumns2linkIndeces;
    std::vector< int > linkIndeces2regrColumns;
    std::vector< std::string > fake_links_names;

    //Robot state
    KDL::JntArray q;
    KDL::JntArray dq;
    KDL::JntArray ddq;

    KDL::Twist kinematic_base_velocity;
    KDL::Twist kinematic_base_acceleration; /**< KDL acceleration: spatial proper acceleration */




    //measured joint torques
    KDL::JntArray measured_torques;

    //Link kinematic quantities
    std::vector<KDL::Frame> X_dynamic_base; /**< for each link store the frame X_kinematic_base_link of the position of a link with respect to the dynamic base */
    std::vector<KDL::Twist> v;
    std::vector<KDL::Twist> a;

    //Vector of subregressors
    //The actual objects are mantained in two different vectors
    //but this vector of pointers is necessary to mantain the serializations
    std::vector<DynamicRegressorInterface *> regressors_ptrs;

    std::vector<baseDynamicsRegressor *> base_regressors;
    std::vector<subtreeBaseDynamicsRegressor *> subtree_regressors;
    std::vector<torqueRegressor *> torque_regressors;

    //Options for regressors
    bool consider_ft_offset; //if true, consider the offset of the FT sensors as parameters

    bool verbose;

    /** \todo Buffers to avoid dynamic memory allocation, remove them using proper stuff */
    int updateBuffers();
    Eigen::MatrixXd& block(int start_row, int arg2, int getNrOfOutputs, int getNrOfParameters);
    Eigen::MatrixXd one_rows_buffer;
    Eigen::MatrixXd six_rows_buffer;
    Eigen::VectorXd one_rows_vector;
    Eigen::VectorXd six_rows_vector;

    //utility function for generating a random regressor for numerical base parameter calculation
    //Given n_samples, the Y (n_samples*getNrOfOutputs() X getNrOfParameters() ) regressor is obtained by stacking the n_samples generated regressors
    //This function returns Y^T Y (getNrOfParameters() X getNrOfParameters() ) (that share the row space with Y)
    int generate_random_regressors(Eigen::MatrixXd & output_matrix,
                                   const bool static_regressor = false,
                                   const bool fixed_base = false,
                                   const KDL::Vector grav_direction=KDL::Vector(0.0,0.0,9.8),
                                   int n_samples = 1000, const bool verbose = false);

    int generate_random_regressors(Eigen::MatrixXd & output_matrix,
                                   const bool static_regressor,
                                   const bool fixed_base,
                                   const KDL::Vector grav_direction,
                                   std::vector<int> fixed_dofs,
                                   std::vector<double> fixed_dofs_values,
                                   int n_samples, const bool verbose);

};

}

}

}


#endif
