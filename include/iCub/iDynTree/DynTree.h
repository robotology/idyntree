/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version)
 *
 */


/**
 * \defgroup iDynTree iDynTree 
 *    
 * @ingroup codyco_modules
 *
 * \note <b>SI units adopted</b>: meters for lengths and radians
 *       for angles.
 *
 * \section dep_sec Dependencies 
 * - KDL
 * 
 * \section intro_sec Description
 * 
 * iDynTree is designed to be an efficient, generic and easy to use library
 * to calculate joint torques and external wrenches given kinetic information
 * (linear acceleration, angular velocity and  angular acceleration of a link, 
 * joint positions, velocities, acceleration) and embedded FT sensor measures, 
 * implementing the techniques described in this two papers:
 * 
 *     - [1] S. Ivaldi, M. Fumagalli, M. Randazzo, F. Nori, G. Metta, and G. Sandini
 *           Computing robot internal/external wrenches by means of inertial, tactile and f/t sensors: theory and implementation on the icub
 *           in Proc. of the 11th IEEE-RAS International Conference on Humanoid Robots, Bled, Slovenia, 2011.
 *           http://people.liralab.it/iron/Papers/conference/780_Ivaldi_etal2011.pdf
 *           http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6100813
 * 
 *     - [2] A. Del Prete, L. Natale, F. Nori, and G. Metta,
 *           Contact Force Estimations Using Tactile Sensors and Force / Torque Sensors
 *           in Human Robot Interaction, 2012, pp. 0–2,
 *           http://pasa.liralab.it/pasapdf/1113_DelPrete_etal2012.pdf
 * 
 * Additional details (some not implemented, like multiple IMUs):  
 * 
 *     - [3] M. Fumagalli, S. Ivaldi, M. Randazzo, L. Natale, G. Metta, G. Sandini, and F. Nori, 
 *           Force feedback exploiting tactile and proximal force/torque sensing
 *           in Autonomous Robots, vol. 33, no. 4, pp. 381–398, 2012.
 *           http://dx.doi.org/10.1007/s10514-012-9291-2
 *           http://people.liralab.it/iron/Papers/journal/IvaldiFumagallietAl.pdf 
 * 
 * \section tested_os_sec Tested OS
 * 
 * Linux
 *
 * \section example_sec Example
 *
 * Exe
 *
 *
 * \author Silvio Traversaro
 * 
 * Copyright (C) 2013 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0.
 * 
 * 
 **/ 

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <iCub/skinDynLib/dynContactList.h>

#include <kdl_codyco/treeserialization.hpp>
#include <kdl_codyco/treepartition.hpp>
#include <kdl_codyco/treegraph.hpp>
#include <kdl_codyco/momentumjacobian.hpp>

#include <kdl_codyco/ftsensor.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include <iostream>
#ifndef __DYNTREE_H__
#define __DYNTREE_H__

namespace iCub
{

namespace iDynTree
{    
    
/**
 * \ingroup iDynTree
 *
 * An class for calculating the torques and the external wrenches
 * in a Rigid Body Dynamic tree, using kinetic information (linear acceleration, angular velocity and
 * angular acceleration of one arbitrary link, joint positions, velocities, acceleration)
 * and FT sensor measures.  * 
 * 
 * \warning The used velocities accelerations, without the information 
 *          about base linear velocity, are not the "real" ones. Are just
 *          the velocities assuming no base linear velocity, that by Galilean
 *          Relativity generate the same dynamics of the "real" ones
 * 
 */
class DynTree  {
    private:
        KDL::CoDyCo::TreeGraph tree_graph; /**< TreeGraph object: it encodes the TreeSerialization and the TreePartition */
        KDL::CoDyCo::TreePartition partition; /**< TreePartition object explicit present as it is conventient to encode/decode dynContact objects */
        
        //Violating DRY principle, but for code clarity 
        int NrOfDOFs;
        int NrOfLinks;
        int NrOfFTSensors;
        int NrOfDynamicSubGraphs;
        
        //state of the robot
        KDL::Frame world_base_frame; /**< the position of the floating base frame with respect to the world reference frame \f$ {}^w H_b \f$ */
        
        KDL::JntArray q;
        KDL::JntArray dq;
        KDL::JntArray ddq;
        
        KDL::Twist imu_velocity;
        KDL::Twist imu_acceleration; /**< KDL acceleration: spatial proper acceleration */
        
        //joint position limits 
        KDL::JntArray q_min;
        KDL::JntArray q_max;
        std::vector<bool> constrained; /**< true if the DOF is subject to limit check, false otherwise */
        
        int constrained_count; /**< the number of DOFs that are constrained */
        
        double setAng(const double q, const int i);
        
        //dynContact stuff 
        std::vector< iCub::skinDynLib::dynContactList > contacts; /**< a vector of dynContactList, one for each dynamic subgraph */
        
        //Sensors measures
        std::vector< KDL::Wrench > measured_wrenches;
        KDL::CoDyCo::FTSensorList ft_list;
        
        //Index representation of the Kinematic tree and the dynamics subtrees
        KDL::CoDyCo::Traversal kinematic_traversal;
        KDL::CoDyCo::Traversal dynamic_traversal;
        
        //Joint quantities
        KDL::JntArray torques;
        
        //Link quantities
        std::vector<KDL::Twist> v;
        std::vector<KDL::Twist> a;
        
        //External forces
        std::vector<KDL::Wrench> f_ext; /**< External wrench acting on a link */
        
        std::vector<KDL::Wrench> f; /**< For a link the wrench transmitted from the link to its parent in the dynamical traversal \warning it is traversal dependent */
        std::vector<KDL::Wrench> f_gi; /**< Gravitational and inertial wrench acting on a link */
        
        //DynTreeContact data structures
        std::vector<int> link2subgraph_index; /**< for each link, return the correspondent dynamics subgraph index */
        std::vector<bool> link_is_subgraph_root; /**< for each link, return if it is a subgraph root */
        std::vector<int> subgraph_index2root_link; /**< for each subgraph, return the index of the root */
        bool are_contact_estimated;
        
        int getSubGraphIndex(int link_index) { return link2subgraph_index[link_index]; }
        bool isSubGraphRoot(int link_index) { return link_is_subgraph_root[link_index]; }
        
        int buildSubGraphStructure(const std::vector<std::string> & ft_names);
        
        /**
         * Get the A e b local to a link, querying the contacts list and the FT sensor list
         * 
         */
        yarp::sig::Vector getLinkLocalAb_contacts(int global_index, yarp::sig::Matrix & A, yarp::sig::Vector & b); 
        
        bool isFTsensor(const std::string & joint_name, const std::vector<std::string> & ft_sensors) const;
        std::vector<yarp::sig::Matrix> A_contacts; /**< for each subgraph, the A regressor matrix of unknowns \todo use Eigen */
        std::vector<yarp::sig::Vector> b_contacts; /**< for each subgraph, the b vector of known terms \todo use Eigen */
        std::vector<yarp::sig::Vector> x_contacts; /**< for each subgraph, the x vector of unknowns */
        
        std::vector<KDL::Wrench> b_contacts_subtree; /**< for each link, the b vector of known terms of the subtree starting at that link expressed in the link frame*/
        
        /**
         * Preliminary version. If there are performance issues, this function
         * has several space for improvement.
         * 
         */
        void buildAb_contacts();
        
        /**
         * 
         * 
         */
        void store_contacts_results();
        
        /**
         * For a given link, returns the sum of the measured wrenches acting on the link (i.e. the sum of the wrenches acting 
         * on the link measured by the FT sensors acting on the link) expressed in the link reference frame
         * 
         */
        KDL::Wrench getMeasuredWrench(int link_id);

        //end DynTreeContact data structures
        
        //Position related quantites
        mutable bool is_X_dynamic_base_updated;
        mutable std::vector<KDL::Frame> X_dynamic_base; /**< for each link store the frame X_dynamic_base_link of the position of a link with respect to the dynamic base */
        
        
        //Debug
        int verbose;
        
        //Buffer 
        yarp::sig::Matrix _H_w_b;
        
        //Jacobian related quantities
        //all this variable are defined once in the class to avoid dynamic memory allocation at each method call
        KDL::Jacobian rel_jacobian; /**< dummy variable used by getRelativeJacobian */
        KDL::CoDyCo::Traversal rel_jacobian_traversal;
        KDL::Jacobian abs_jacobian; /**< dummy variable used by getJacobian */
        
        //COM related quantities
        KDL::Jacobian com_jacobian;
        KDL::CoDyCo::MomentumJacobian momentum_jacobian;
        KDL::Jacobian com_jac_buffer;
        KDL::CoDyCo::MomentumJacobian momentum_jac_buffer;
        yarp::sig::Vector com_yarp;
        std::vector<KDL::Vector> subtree_COM;
        std::vector<double> subtree_mass;
        KDL::RigidBodyInertia total_inertia;

        
    public:
        DynTree();
 
        void constructor(const KDL::Tree & _tree, const std::vector<std::string> & joint_sensor_names, const std::string & imu_link_name, 
                         KDL::CoDyCo::TreeSerialization  serialization=KDL::CoDyCo::TreeSerialization(), KDL::CoDyCo::TreePartition partition=KDL::CoDyCo::TreePartition(), std::vector<KDL::Frame> parent_sensor_transforms=std::vector<KDL::Frame>(0));

    
        /**
         * Constructor for DynTree
         * 
         * @param _tree the KDL::Tree that must be used
         * @param joint_sensor_names the names of the joint that should 
         *        be considered as FT sensors
         * @param imu_link_name name of the link considered the IMU sensor
         * @param serialization (optional) an explicit serialization of tree links and DOFs
         * @param partition (optional) a partition of the tree (division of the links and DOFs in non-overlapping sets)
         *
         */
        DynTree(const KDL::Tree & _tree, const std::vector<std::string> & joint_sensor_names, const std::string & imu_link_name, KDL::CoDyCo::TreeSerialization  serialization=KDL::CoDyCo::TreeSerialization(), KDL::CoDyCo::TreePartition partition=KDL::CoDyCo::TreePartition());
        
        ~DynTree();
        
        /**
         * Get the number of (internal) degrees of freedom of the tree
         *
         * \note This function returns only the internal degrees of freedom of the robot (i.e. not counting the 6
         *       DOFs of the floating base
         * 
         */
        int getNrOfDOFs(const std::string & part_name="");
        
        /**
         * Get the number of links of the tree
         *
         */
        int getNrOfLinks();
        
        /**
         * Get the global index for a link, given a link name
         * @param link_name the name of the link
         * @return an index between 0..getNrOfLinks()-1 if all went well, -1 otherwise
         */
        int getLinkIndex(const std::string & link_name);
       
        /**
         * Get the global index for a DOF, given a DOF name
         * @param dof_name the name of the dof
         * @return an index between 0..getNrOfDOFs()-1 if all went well, -1 otherwise
         *
         */
        int getDOFIndex(const std::string & dof_name);
        
        
         /**
         * Get the global index for a link, given a part and a part local index
         * @param part_id the id of the part
         * @param local_link_index the index of the link in the given part
         * @return an index between 0..getNrOfLinks()-1 if all went well, -1 otherwise
         */
        int getLinkIndex(const int part_id, const int local_link_index);
       
        /**
         * Get the global index for a DOF, given a part id and a DOF part local index
         * @param part_id the id of the part
         * @param local_DOF_index the index of the DOF in the given part
         * @return an index between 0..getNrOfDOFs()-1 if all went well, -1 otherwise
         *
         */
        int getDOFIndex(const int part_id, const int local_DOF_index);
        
        /**
         * Get the global index for a link, given a part and a part local index
         * @param part_name the name of the part
         * @param local_link_index the index of the link in the given part
         * @return an index between 0..getNrOfLinks()-1 if all went well, -1 otherwise
         */
        int getLinkIndex(const std::string & part_name, const int local_link_index);
       
        /**
         * Get the global index for a DOF, given a part id and a part local index
         * @param part_name the name of the part
         * @param local_DOF_index the index of the DOF in the given part
         * @return an index between 0..getNrOfDOFs()-1 if all went well, -1 otherwise
         *
         */
        int getDOFIndex(const std::string & part_name, const int local_DOF_index);
    
        /**
        * Set the rototranslation between the world and the base reference
        * frames, expressed in the world reference frame \f$ {}^wH_b \f$ 
        * 
        * @param H_w_p a 4x4 rototranslation matrix
        * @return true if all went well, false otherwise (a problem in the input)
        */
        bool setWorldBasePose(const yarp::sig::Matrix & H_w_p);
        
        /**
        * Get the rototranslation between the world and the base reference
        * frames, expressed in the world reference frame \f$ {}^wH_b \f$ 
        * 
        * @return H_w_p a 4x4 rototranslation matrix
        */
        yarp::sig::Matrix getWorldBasePose();
        
        /**
        * Set joint positions in the specified part (if no part 
        * is specified, set the joint positions of all the tree)
        * @param _q vector of joints position
        * @param part_name optional: the name of the part of joint to set
        * @return the effective joint positions, considering min/max values
        */
        virtual yarp::sig::Vector setAng(const yarp::sig::Vector & _q, const std::string & part_name="") ;
        
        /**
        * Get joint positions in the specified part (if no part 
        * is specified, get the joint positions of all the tree)
        * @param part_name optional: the name of the part of joints to set
        * @return vector of joint positions
        */
        virtual yarp::sig::Vector getAng(const std::string & part_name="") const;
        
        /**
        * Set joint speeds in the specified part (if no part 
        * is specified, set the joint speeds of all the tree)
        * @param _q vector of joint speeds
        * @param part_name optional: the name of the part of joints to set
        * @return the effective joint speeds, considering min/max values
        */
        virtual yarp::sig::Vector setDAng(const yarp::sig::Vector & _q, const std::string & part_name="");
        
        /**
        * Get joint speeds in the specified part (if no part 
        * is specified, get the joint speeds of all the tree)
        * @param part_name optional: the name of the part of joints to get
        * @return vector of joint speeds
        * 
        * \note please note that this does returns a vector of size getNrOfDOFs()
        */
        virtual yarp::sig::Vector getDAng(const std::string & part_name="") const;
        
        /**
        * Set joint accelerations in the specified part (if no part 
        * is specified, set the joint accelerations of all the tree)
        * @param _q vector of joint speeds
        * @param part_name optional: the name of the part of joints to set
        * @return the effective joint accelerations, considering min/max values
        */
        virtual yarp::sig::Vector setD2Ang(const yarp::sig::Vector & _q, const std::string & part_name="");
        
        /**
        * Get joint speeds in the specified part (if no part 
        * is specified, get the joint speeds of all the tree)
        * @param part_name optional: the name of the part of joints to get
        * @return vector of joint accelerations
        */
        virtual yarp::sig::Vector getD2Ang(const std::string & part_name="") const;
        
    
        /**
        * Set the inertial sensor measurements 
        * @param w0 a 3x1 vector with the initial/measured angular velocity
        * @param dw0 a 3x1 vector with the initial/measured angular acceleration
        * @param ddp0 a 3x1 vector with the initial/measured 3D proper (with gravity) linear acceleration
        * @return true if succeeds (correct vectors size), false otherwise
        */
        virtual bool setInertialMeasure(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0);
        
        /**
        * Get the inertial sensor measurements 
        * @param w0 a 3x1 vector with the initial/measured angular velocity
        * @param dw0 a 3x1 vector with the initial/measured angular acceleration
        * @param ddp0 a 3x1 vector with the initial/measured 3D proper (with gravity) linear acceleration
        * @return true if succeeds (correct vectors size), false otherwise
        */
        virtual bool getInertialMeasure(yarp::sig::Vector &w0, yarp::sig::Vector &dw0, yarp::sig::Vector &ddp0) const;
    
        /**
        * Set the FT sensor measurements on the specified sensor 
        * @param sensor_index the code of the specified sensor
        * @param ftm a 6x1 vector with forces (0:2) and moments (3:5) measured by the FT sensor
        * @return true if succeeds, false otherwise
        * 
        * \warning The convention used to serialize the wrench (Force-Torque) is different
        *          from the one used in Spatial Algebra (Torque-Force)
        * 
        */
        virtual bool setSensorMeasurement(const int sensor_index, const yarp::sig::Vector &ftm);
        
        /**
        * Get the FT sensor measurements on the specified sensor 
        * @param sensor_index the code of the specified sensor
        * @param ftm a 6x1 vector with forces (0:2) and moments (3:5) measured by the FT sensor
        * @return true if succeeds, false otherwise
        * 
        * \warning The convention used to serialize the wrench (Force-Torque) is different
        *          from the one used in Spatial Algebra (Torque-Force)
        * 
        * \note if dynamicRNEA() is called without before calling estimateContactForces() this
        *       function retrives the "simulated" measure of the sensor from the
        *       RNEA backward propagation of wrenches   
        */
        virtual bool getSensorMeasurement(const int sensor_index, yarp::sig::Vector &ftm) const;
        

                        
        /** @name Methods to deal with joint constraints
        *  Activate, set and get constraints on joint values
        */
        //@{
        
        /**
         * Returns a list containing the min value for each joint. 
         */
        virtual yarp::sig::Vector getJointBoundMin(const std::string & part_name="");
        
        /**
         * Returns a list containing the max value for each joint.
         */
        virtual yarp::sig::Vector getJointBoundMax(const std::string & part_name="");
        
        /**
         * Set a list containing the min value for each joint. 
         */
        virtual bool setJointBoundMin(const yarp::sig::Vector & _q, const std::string & part_name="");
        
        /**
         * Set a list containing the max value for each joint.
         */
        virtual bool setJointBoundMax(const yarp::sig::Vector & _q, const std::string & part_name="");

       /**
        * Sets the constraint status of all chain links.
        * @param _constrained is the new constraint status. 
        */
        virtual void setAllConstraints(bool _constrained);

       /**
        * Sets the constraint status of ith link.
        * @param _constrained is the new constraint status. 
        */
       virtual void setConstraint(unsigned int i, bool _constrained);

       /**
        * Returns the constraint status of ith link. 
        * @return current constraint status.
        */
        virtual bool getConstraint(unsigned int i);


        
        //@}
        
        /** @name Methods to execute phases of RNEA
        *  Methods to execute phases of Recursive Newton Euler Algorithm
        */
        //@{
        
        
        /**
        * Execute a loop for the calculation of the rototranslation between every frame
        * The result of this computations can be then called using getPosition() methods
        * @return true if succeeds, false otherwise
        */
        virtual bool computePositions() const;  
        
        /**
        * Execute the kinematic phase (recursive calculation of position, velocity,
        * acceleration of each link) of the RNE algorithm.
        * @return true if succeeds, false otherwise
        */
        virtual bool kinematicRNEA();    
        
        /**
        * Estimate the external contacts, supplied by the setContacts call
        * for each dynamical subtree
        * 
        */
        virtual bool estimateContactForces();
        
        /**
        * Execute the dynamical phase (recursive calculation of internal wrenches
        * and of torques) of the RNEA algorithm for all the tree. 
        * @return true if succeeds, false otherwise
        */
        virtual bool dynamicRNEA();
        
        //@}
        
        /** @name Get methods for output quantities
        *  Methods to get output quantities
        */
        //@{
    
        /**
        * Get the 4x4 rototranslation matrix of a link frame with respect to the world frame ( \f$ {}^wH_i \f$)
        * @param link_index the index of the link 
        * @param inverse if true, return the rototranslation of the world frame with respect to the link frame ( \f$ {}^iH_w \f$ , defaul: false )  
        * @return a 4x4 rototranslation yarp::sig::Matrix 
        */
        virtual yarp::sig::Matrix getPosition(const int link_index, bool inverse = false) const;
        
        /**
        * Get the 4x4 rototranslation matrix between two link frames 
        * (in particular, of the second link frame expressed in the first link frame, \f$ {}^fH_s \f$))
        * @param first_link the index of the first link 
        * @param second_link the index of the second link
        * @return a 4x4 rototranslation yarp::sig::Matrix
        */
        virtual yarp::sig::Matrix getPosition(const int first_link, const int second_link) const;
        
        /**
        * Get the velocity of the specified link, expressed in the world reference frame, but using as reference point
        * the origin of the link local reference frame
        * @param link_index the index of the link 
        * @param if true, return the velocity expressed in the link local frame
        * @return a 6x1 vector with linear velocity \f$ {}^wv_i \f$ (0:2) and angular velocity \f$ {}^w\omega_i\f$ (3:5)
        */
        virtual yarp::sig::Vector getVel(const int link_index, bool local=false) const;
    
        /**
        * Get the acceleration of the specified link, expressed in the link local reference frame
        * @param link_index the index of the link 
        * @return a 6x1 vector with linear acc \f$ {}^ia_i \f$(0:2) and angular acceleration \f$ {}^i\dot{\omega}_i \f$ (3:5)
        *
        * \note This function returns the classical linear acceleration, not the spatial one
        */
        virtual yarp::sig::Vector getAcc(const int link_index) const;
    
        
        /**
        * Get joint torques in the specified part (if no part 
        * is specified, get the joint torques of all the tree)
        * @param part_name optional: the name of the part of joints to get
        * @return vector of joint torques
        */
        virtual yarp::sig::Vector getTorques(const std::string & part_name="") const;
        
    
        //@}
        /** @name Methods related to contact forces
        *  This methods are related both to input and output of the esimation:
        *  the iCub::skinDynLib::dynContactList is used both to specify the 
        *  unkown contacts via setContacts, and also to get the result of the
        *  estimation via getContacts
        * 
        *  \note If for a given subtree no contact is given, a default concact 
        *  is assumed, for example ad the end effector
        */
        //@{
        
        /**
        * Set the unknown contacts
        * @param contacts the list of the contacts on the DynTree
        * @return true if operation succeeded, false otherwise
        */
        virtual bool setContacts(const iCub::skinDynLib::dynContactList &contacts_list);
        
        /**
        * Get the contacts list, containing the results of the estimation if
        * estimateContacts was called
        * @return A reference to the external contact list
        */
        virtual const iCub::skinDynLib::dynContactList getContacts() const;
        
        //@}
        
        //@}
        /** @name Methods related to Jacobian calculations
        * 
        * 
        */
        //@{
        /**
        * For a floating base structure, outpus a 6x(nrOfDOFs+6) yarp::sig::Matrix \f$ {}^i J_i \f$ such
        * that \f$ {}^w v_i = {}^wJ_i  dq_{fb} \f$
        * where w is the world reference frame and \f$ dq_{fb} \f$ is the floating base velocity vector,
        * where the first 3 elements are \f$ {}^wv_b\f$, the next 3 are \f$ {}^w\omega_b\f$ and the remaining 
        * are the proper joint velocities. 
        * @param link_index the index of the link
        * @param jac the output yarp::sig::Matrix 
        * @param local if true, return \f$ {}^iJ_i \f$ (the Jacobian expressed in the local frame of link i) (default: false)
        * @return true if all went well, false otherwise
        * 
        * \note the link used as a floating base is the base used for the dynamical loop
        * \note {}^w v_i is expressed in the world reference frame, but its reference point is the origin of the frame of link i
        */
        virtual bool getJacobian(const int link_index, yarp::sig::Matrix & jac, bool local=false);
        
        /**
        * Get the 6+getNrOfDOFs() yarp::sig::Vector, characterizing the floating base velocities of the tree
        * @return a vector where the 0:5 elements are the one of the dynamic base expressed in the world frame (the same that are obtained calling
        *         getVel(dynamic_base_index), while the 6:6+getNrOfDOFs()-1 elements are the joint speeds
        */
        virtual yarp::sig::Vector getDQ_fb() const;
        
        //virtual yarp::sig::Vector getD2Q_fb() const;
        
        
        /**
        * For a floating base structure, if d is the distal link index and b is the jacobian base link index 
        * outputs a 6x(nrOfDOFs) yarp::sig::Matrix \f$ {}^d J_{b,d} \f$ such
        * that \f[ {}^d v_d = {}^dJ_{b,d}  \dot{q} + {}^d v_b \f]
        * @param jacobian_distal_link the index of the distal link
        * @param jacobian_base_link the index of the base link
        * @param jac the output yarp::sig::Matrix 
        * @param global if true, return \f$ {}^wJ_{s,f} \f$ (the Jacobian expressed in the world frame) (default: false)
        * @return true if all went well, false otherwise
        */
        virtual bool getRelativeJacobian(const int jacobian_distal_link, const int jacobian_base_link, yarp::sig::Matrix & jac, bool global=false);
        
        //@}
        /** @name Methods related to Center of Mass calculation
        * 
        * 
        */
        //@{
        
        /**
        * Get Center of Mass of the specified part (if no part 
        * is specified, get the COM of all the tree) expressed
        * in the world frame 
        * @param part_name optional: the name of the part of joints to get
        * @return Center of Mass vector
        * 
        * \todo prepare a suitable interface for specifing both an arbitrary part and a arbitrary frame of expression
        */
        virtual yarp::sig::Vector getCOM(const std::string & part_name="", const int link_index = -1);
        
        /**
        * Get Center of Mass Jacobian of the specified part (if no part 
        * is specified, get the Jacobian of the COM of all the tree) expressed in
        * the world frame
        * @param jac the output jacobiam matrix
        * @param part_name optional: the name of the part of joints to get
        * @return true if succeeds, false otherwise
        */
        virtual bool getCOMJacobian(yarp::sig::Matrix & jac, const std::string & part_name="");
        
        /**
         * Temporary function, do not use.
         * 
         */
        virtual bool getCOMJacobian(yarp::sig::Matrix & jac, yarp::sig::Matrix & momentum_jac, const std::string & part_name="");

        
        /**
        * Get Velocity of the Center of Mass of the specified part (if no part 
        * is specified, get the velocity of the center of mass of all tree) expressed
        * in the world frame 
        * @param part_name optional: the name of the part of joints to get
        * @return velocity of Center of Mass vector
        */
        yarp::sig::Vector getVelCOM();
        
        yarp::sig::Vector getMomentum();

        //@}
        
        /** @name Methods related to inertial parameters regressor 
        * 
        * 
        */
        //@{
        /**
        * Get the dynamics regressor, such that dynamics_regressor*dynamics_parameters 
        * return a 6+nrOfDOFs vector where the first six elements are the components of the base wrench,
        * while the other nrOfDOFs elements are the torques 
        * 
        * @return a 6+nrOfDOFs x 10*nrOfLinks yarp::sig::Matrix
        */
        virtual bool getDynamicsRegressor(yarp::sig::Matrix & mat);
        
        /**
        * Get the dynamics parameters currently used for the dynamics calculations
        * 
        * @return a 10*nrOfLinks yarp::sig::Vector
        */
        virtual bool getDynamicsParameters(yarp::sig::Vector & vet);
        //@} 

        KDL::Tree getKDLTree() { return tree_graph.getTree(); }
        
        KDL::CoDyCo::TreeGraph getKDLUndirectedTree() { return tree_graph; }
    
};

}//end namespace

}

#endif
