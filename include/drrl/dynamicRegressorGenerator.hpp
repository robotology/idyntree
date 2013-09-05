/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
     
#ifndef KDL_CODYCO_DYNREGRESSORGENERATOR_HPP
#define KDL_CODYCO_DYNREGRESSORGENERATOR_HPP

#include <kdl/tree.hpp>

#include <kdl_codyco/utils.hpp>

#include <kdl_codyco/treegraph.hpp>

#include <kdl_codyco/ftsensor.hpp>


namespace DRRL 
{    
    
//typedef FixedParameterIndex int;
//typedef UnknownParameterIndex int;
typedef ParameterIndex int;

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

    /**
    * Constructor for DynRegressorGenerator
    * 
    * \note the dynamics base, used to define forward/backward direction of torque regressor, is
    *       the root of the KDL::Tree
    * 
    * @param tree the KDL::Tree description of the robot structure
    * @param kinematic_base the link used as the root for propagation of kinematic information
    * @param ft_sensor_names the names of the Joints (of type Joint::None) that are Force Torque sensors
    * 
    */
    DynamicRegressorGenerator(KDL::Tree & tree, std::string kinematic_base, std::vector< std::string > & ft_sensor_names=std::vector< std::string >(0), bool ft_sensor_offset=true,KDL::CoDyCo::TreeSerialization serialization=KDL::CoDyCo::TreeSerialization());
    
    ~DynamicRegressorGenerator() {};
    
    void addRegressorRows(DynamicRegressorInterface & dynamic_regressor);
    
    /**
    * 
    * Get the number of parameters used by the regressor currently generated 
    * (default: 10*tree.getNrOfSegments(), then depending on the type of regressor more can be added) 
    * 
    */
    int getNrOfParameters();
    
    int getNrOfOutputs();
    
    //The feature of fixing/unfixing parameters would be implemented in a later version
    //int getNrOfUnknownParameters();
    
    //int getNrOfFixedParameters();
    
    //fixParameter(ParameterIndex par_index, double parameter_value);
    //fixParameter(FixedParameterIndex fixed_par_index, double 
    //double getFixedParameter(FixedParameterIndex fixed_par_index);
    //unfixParameter(ParameterIndex par_index);
    
    
    std::string getDescriptionOfParameter(int parameter_index);
    
    /**
    * Get a human readable description of the considered parameters 
    * 
    * @return a std::string containg the description of the parameters
    */
    std::string getDescriptionOfParameters();
    
    //std::string getDescriptionOfFixedParameters();
    
    //std::string getDescriptionOfUnknownParameters();
    
    /**
     * Set the state for the robot (floating base)
     * 
     */       
    int setRobotState(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Twist& base_velocity, const Twist& base_acceleration);

    /**
     * Set the state for the robot (fixed base)
     *
     */
    int setRobotState(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Twist& base_gravity);

    
    //Eventually the call would be only setSensor and the sensor type 
    //subsystem would be more flexible, for now we do in this way
    
    /**
     * 
     * @param ft_sensor_index the index of the FT sensor, an integer from 0 to NrOfFTSensors-1
     */
    setFTSensorMeasurement(int ft_sensor_index, const KDL::Wrench ftm);
    
    /**
     * 
     * @param dof_index the index of the degree of freedom on which the torque was measured, an interger from 0 to NrOfDOFs-1
     */
    setTorqueSensorMeasurement(int junction_index, double measure);
    
    
    /**
    * Return the compute regressor and the corresponding output (calculate from sensor readings)
    * @param regressor a getNrOfOutputs() times getNrOfUnknownParameters() Matrix 
    * @param known_terms a getNrOfOutputs() parameters Vector
    */
    int computeRegressor(MatrixXd & regressor, VectorXd & known_terms);
            
private:
    KDL::CoDyCo::TreeGraph tree_graph; /**< TreeGraph object: it encodes the TreeSerialization and the TreePartition */
    
    KDL::CoDyCo::Traversal dynamic_traversal; 
    KDL::CoDyCo::Traversal kinematic_traversal; /**< Traversal object: defining the kinematic base of the tree */
    
    //Violating DRY principle, but for code clarity 
    int NrOfDOFs;
    int NrOfLinks;
    int NrOfFTSensors;
    int NrOfParameters;
    int NrOfOutputs;
    
    //Robot state
    
    KDL::JntArray q;
    KDL::JntArray dq;
    KDL::JntArray ddq;
    
    KDL::Twist kinematic_base_velocity;
    KDL::Twist kinematic_base_acceleration; /**< KDL acceleration: spatial proper acceleration */
    
    
    //measured 6 axis Force/torques
    std::vector< KDL::Wrench > measured_wrenches;
    KDL::CoDyCo::FTSensorList ft_list;
    
    //measured joint torques
    KDL::JntArray measured_torques;
    
    //Link kinematic quantities
    std::vector<KDL::Frame> X_dynamic_base; /**< for each link store the frame X_kinematic_base_link of the position of a link with respect to the dynamic base */
    std::vector<KDL::Twist> v;
    std::vector<KDL::Twist> a;
    
    //Options for regressors
    bool consider_ft_offset; //if true, consider the offset of the FT sensors as parameters
    
    bool verbose;
        
    
        
}
}

             
#endif
