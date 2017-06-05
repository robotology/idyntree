/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version)
 *
 */

#ifndef IDYNTREE_H
#define IDYNTREE_H

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include <kdl_codyco/treeserialization.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include <kdl_codyco/momentumjacobian.hpp>
#include <kdl_codyco/floatingjntspaceinertiamatrix.hpp>

#include "iDynTree/Sensors/Sensors.h"

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include <iostream>
#include <vector>
#include <map>


namespace iCub
{

namespace iDynTree
{

const int WORLD_FRAME = -10;
const int DEFAULT_INDEX_VALUE = -20;

class skinDynLibLinkID {
     public:
          int body_part;
          int local_link_index;

    bool operator<(const skinDynLibLinkID& k) const
    {
        if(this->body_part < k.body_part)
        {
           return true;
        }
        else if(this->body_part > k.body_part)
        {
           return false;
        }
        else
        {
            return this->local_link_index < k.local_link_index;
        }
    }

    bool operator==(const skinDynLibLinkID& k) const
    {
        return (this->body_part == k.body_part &&
                this->local_link_index == k.local_link_index);
    }
};

class iDynTreeLinkAndFrame {
     public:
          int link_index;
          int frame_index;
};



/**
 * \ingroup iDynTreeYARP
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
    protected:
        /** configured flag: if true the model was correctly loaded
         *  if false not an all the methods will return false */
        bool configured;

        KDL::CoDyCo::UndirectedTree undirected_tree; /**< UndirectedTree object: it encodes the TreeSerialization and the TreePartition */

        //Violating DRY principle, but for code clarity
        unsigned NrOfDOFs;
        unsigned NrOfLinks;
        unsigned NrOfFTSensors;
        unsigned NrOfDynamicSubGraphs;

        //state of the robot

        KDL::Frame world_base_frame; /**< the position of the floating base frame with respect to the world reference frame \f$ {}^w H_b \f$ */


        KDL::JntArray q;
        KDL::JntArray dq;
        KDL::JntArray ddq;

        KDL::Twist imu_velocity;

        KDL::Twist imu_acceleration;  /**< KDL acceleration: spatial proper acceleration */



        //joint position limits
        KDL::JntArray q_jnt_min;
        KDL::JntArray q_jnt_max;
        std::vector<bool> constrained; /**< true if the DOF is subject to limit check, false otherwise */

        //joint torque limits
        KDL::JntArray tau_max;

        unsigned constrained_count; /**< the number of DOFs that are constrained */

        double setAng(const double q, const int i);


        //Sensors measures
        //std::vector< KDL::Wrench > measured_wrenches;
        //KDL::CoDyCo::FTSensorList ft_list;
        ::iDynTree::SensorsMeasurements sensor_measures;
        std::vector<std::string> m_joint_sensor_names;


        //Index representation of the Kinematic tree and the dynamics subtrees
        KDL::CoDyCo::Traversal kinematic_traversal;
        KDL::CoDyCo::Traversal dynamic_traversal;

        //Joint quantities
        KDL::JntArray torques;

        //Link quantities
        std::vector<KDL::Twist> v;
        std::vector<KDL::Twist> a;

        //Base force, calculated as output of the dynamic RNEA (expressed in the base reference frame)
        KDL::Wrench base_residual_f;

        //External forces
        std::vector<KDL::Wrench> f_ext; /**< External wrench acting on a link */

        std::vector<KDL::Wrench> f; /**< For a link the wrench transmitted from the link to its parent in the dynamical traversal \warning it is traversal dependent */
        std::vector<KDL::Wrench> f_gi; /**< Gravitational and inertial wrench acting on a link */



        //Position related quantites
        mutable bool is_X_dynamic_base_updated;
        mutable std::vector<KDL::Frame> X_dynamic_base; /**< for each link store the frame X_dynamic_base_link of the position of a link with respect to the dynamic base */
        mutable bool is_X_world_updated;
        mutable std::vector<KDL::Frame> X_world; /**< for each link store the frame world_X_link of the position of a link with respect to the world */


        //Debug
        int verbose;

        //Buffers
        yarp::sig::Matrix _H_w_b;
        yarp::sig::Matrix com_jacobian_buffer;

        //Generic 3d buffer
        mutable yarp::sig::Vector com_yarp;
        mutable yarp::sig::Vector three_zeros;

        mutable yarp::sig::Vector sixteen_double_zero;
        mutable KDL::Frame error_frame;

        //Jacobian related quantities
        //all this variable are defined once in the class to avoid dynamic memory allocation at each method call
        KDL::Jacobian rel_jacobian; /**< dummy variable used by getRelativeJacobian */
        KDL::CoDyCo::Traversal rel_jacobian_traversal;
        KDL::Jacobian abs_jacobian; /**< dummy variable used by getJacobian */

        //COM related quantities
        KDL::Jacobian m_com_jacobian;
        KDL::CoDyCo::MomentumJacobian m_momentum_jacobian;
        KDL::Jacobian m_com_jac_buffer;
        KDL::CoDyCo::MomentumJacobian m_momentum_jac_buffer;
        mutable std::vector<KDL::Vector> m_subtree_COM;
        mutable std::vector<double> m_subtree_mass;
        KDL::RigidBodyInertia m_total_inertia;

        //MassMatrix
        KDL::CoDyCo::FloatingJntSpaceInertiaMatrix fb_jnt_mass_matrix;
        std::vector<KDL::RigidBodyInertia> subtree_crbi;


        //Map for correctly deal with skinDynLib IDs

        std::map<skinDynLibLinkID,iDynTreeLinkAndFrame> skinDynLibLinkMap;

                bool are_contact_estimated;



    static bool loadJointLimitsFromURDFFile(std::string urdfFile, KDL::CoDyCo::UndirectedTree undirectedTree, yarp::sig::Vector &yarpJointMinLimit, yarp::sig::Vector &yarpJointMaxLimit);


    public:
        DynTree();

        void constructor(const KDL::Tree & _tree,
                         const std::vector<std::string> & joint_ft_sensor_names,
                         const std::string & imu_link_name,
                         KDL::CoDyCo::TreeSerialization  serialization=KDL::CoDyCo::TreeSerialization());

        /**
         * Constructor for DynTree
         *
         * @param _tree the KDL::Tree that must be used
         * @param serialization (optional) an explicit serialization of tree links and DOFs
         *
         */
        DynTree(const KDL::Tree & _tree,
                KDL::CoDyCo::TreeSerialization  serialization=KDL::CoDyCo::TreeSerialization());

        /**
         * Constructor for DynTree
         *
         * @param _tree the KDL::Tree that must be used
         * @param joint_sensor_names the names of the joint that should
         *        be considered as FT sensors
         * @param imu_link_name name of the link considered the IMU sensor
         * @param serialization (optional) an explicit serialization of tree links and DOFs
         *
         */
        DynTree(const KDL::Tree & _tree,
                const std::vector<std::string> & joint_sensor_names,
                const std::string & imu_link_name,
                KDL::CoDyCo::TreeSerialization  serialization=KDL::CoDyCo::TreeSerialization());

        /**
         * Constructor for DynTree
         *
         * @param urdf_file the URDF file used for loading the structure of the model
         * @param joint_sensor_names the names of the joint that should
         *        be considered as FT sensors
         * @param imu_link_name name of the link considered the IMU sensor
         * @param serialization (optional) an explicit serialization of tree links and DOFs
         *
         */
        DynTree(const std::string urdf_file,
                const std::vector<std::string> & joint_sensor_names,
                const std::string & imu_link_name,
                KDL::CoDyCo::TreeSerialization  serialization=KDL::CoDyCo::TreeSerialization());

        virtual ~DynTree();

        /**
         * Load a URDF model of the robot in the DynTree class.
         *
         * @param urdf_file_name the path to the urdf file loaded.
         * @return true if all went well (the file was opened and the model correctly loaded)
         *         false otherwise
         */
        bool loadURDFModel(const std::string & urdf_file_name);

        /**
         * Get the number of (internal) degrees of freedom of the tree
         *
         * \note This function returns only the internal degrees of freedom of the robot (i.e. not counting the 6
         *       DOFs of the floating base
         *
         */
        int getNrOfDOFs() const;


        /**
         * Get the number of links of the tree
         *
         */
        int getNrOfLinks() const;

        /**
         * Get the number of frames of the tree
         *
         */
        int getNrOfFrames() const;


        /**
         * Get the number of 6-axis Force Torque sensors
         *
         */
        int getNrOfFTSensors() const;


        /**
         * Get the number of IMUs
         *
         */
        int getNrOfIMUs() const;


        /**
         * Get the global index for a link, given a link name
         * @param link_name the name of the link
         * @return an index between 0..getNrOfLinks()-1 if all went well, -1 otherwise
         */
        int getLinkIndex(const std::string & link_name) const;

        bool getLinkName(const int link_index, std::string & link_name) const;

        int getFrameIndex(const std::string & frame_name) const;

        bool getFrameName(const int frame_index, std::string & frame_name) const;

        /**
         * Get the global index for a DOF, given a DOF name
         * @param dof_name the name of the dof
         * @return an index between 0..getNrOfDOFs()-1 if all went well, -1 otherwise
         *
         */
        int getDOFIndex(const std::string & dof_name) const;

        bool getDOFName(const int dof_index, std::string & dof_name) const;



        /**
         * Get the global index for a junction, given a junction name
         * @param junction_name the name of the dof
         *
         */
        int getJunctionIndex(const std::string & junction_name) const;

        bool getJunctionName(const int junction_index, std::string & junction_name) const;


        /**
         * Get the index of a FT sensor, given the FT junction name.
         * \note for backward compatibility, this function returns the
         * the FT index given the name of the junction at which it is attached.
         */
        int getFTSensorIndex(const std::string & ft_junction_name) const;

        bool getFTSensorName(const int ft_sensor_index, std::string & ft_sensor_name) const;


        /**
         * Get the global index of a IMU, given the IMU frame name
         *
         */
        int getIMUIndex(const std::string & imu_name) const;

        bool getIMUName(const int imu_sensor_index, std::string & imu_name) const;

        /**
        * Set the rototranslation between the world and the base reference
        * frames, expressed in the world reference frame \f$ {}^wH_b \f$
        *
        * @param H_w_p a 4x4 rototranslation matrix
        * @return true if all went well, false otherwise (a problem in the input)
        */
        bool setWorldBasePose(const yarp::sig::Matrix & H_w_p);

        bool setWorldBasePoseKDL(const KDL::Frame & H_w_p);

        /**
        * Get the rototranslation between the world and the base reference
        * frames, expressed in the world reference frame \f$ {}^wH_b \f$
        *
        * @return H_w_p a 4x4 rototranslation matrix
        */
        yarp::sig::Matrix getWorldBasePose();

        KDL::Frame getWorldBasePoseKDL();


        /**
        * Set joint positions in the specified part
        * @param _q vector of joints position
        * @return the actual joint positions, considering min/max values
        */
        virtual yarp::sig::Vector setAng(const yarp::sig::Vector & _q) ;

        virtual bool setAngKDL(const KDL::JntArray & _q) ;

        /**
        * Get joint positions
        * @return vector of joint positions
        */
        virtual yarp::sig::Vector getAng() const;

        virtual bool getAngKDL(KDL::JntArray & q) const;

        /**
        * Set joint speeds
        * @param _q vector of joint speeds
        * @return the effective joint speeds, considering min/max values
        */
        virtual yarp::sig::Vector setDAng(const yarp::sig::Vector & _dq);

        virtual bool setDAngKDL(const KDL::JntArray & _dq) ;


        /**
        * Get joint speeds
        * @return vector of joint speeds
        *
        * \note please note that this does returns a vector of size getNrOfDOFs()
        */
        virtual yarp::sig::Vector getDAng() const;

        virtual bool getDAngKDL(KDL::JntArray & dq) const;

        /**
        * Set joint accelerations
        * @param _q vector of joint speeds
        * @return the effective joint accelerations, considering min/max values
        */
        virtual yarp::sig::Vector setD2Ang(const yarp::sig::Vector & _d2q);

        virtual bool setD2AngKDL(const KDL::JntArray & _d2q) ;

        /**
        * Get joint accelerations
        * @return vector of joint accelerations
        */
        virtual yarp::sig::Vector getD2Ang() const;

        virtual bool getD2AngKDL(KDL::JntArray & d2q) const;

        /**
         * Set the gravity vector.
         * As the setInertialMeasure method, but setting angular velocity and angular acceleration to zero.
         *
         * @param ddp0 a 3x1 vector with the 3d vector of gravity, expressed in the kinematic base frame.
         * @return true if succeeds (correct vectors size), false otherwise
         */
        virtual bool setGravity(const yarp::sig::Vector &gravity);

        /**
        * Set the inertial sensor measurements
        * @param w0 a 3x1 vector with the initial/measured angular velocity
        * @param dw0 a 3x1 vector with the initial/measured angular acceleration
        * @param ddp0 a 3x1 vector with the initial/measured 3D proper (with gravity) linear acceleration
        * @return true if succeeds (correct vectors size), false otherwise
        *
        * \note this variables are considered to be in **kinematic base** reference frame
        */
        virtual bool setInertialMeasure(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0);

        /**
        * Set the inertial sensor measurements
        * @param dp0 a 3x1 vector with the initial/measured 3D proper (with gravity) linear acceleration
        * @param w0 a 3x1 vector with the initial/measured angular velocity
        * @param ddp0 a 3x1 vector with the initial/measured 3D proper (with gravity) linear acceleration
        * @param dw0 a 3x1 vector with the initial/measured angular acceleration
        * @return true if succeeds (correct vectors size), false otherwise
        *
        * \note this variables are considered in **kinematic base** reference frame
        */
        virtual bool setInertialMeasureAndLinearVelocity(const yarp::sig::Vector &dp0, const yarp::sig::Vector &w0, const yarp::sig::Vector &ddp0, const yarp::sig::Vector &dw0);

        /**
         * Set the kinematic base (IMU) velocity and acceleration, expressed in world frame
         * @param base_vel a 6x1 vector with lin/rot velocity (the one that will be returned by getVel(kinematic_base)
         * @param base_classical_acc a 6x1 vector with lin/rot acceleration (the one that will be returned by getAcc(kinematic_base)
         *
         * \note this variables are considered in **world** reference frame
         */
        virtual bool setKinematicBaseVelAcc(const yarp::sig::Vector &base_vel, const yarp::sig::Vector &base_classical_acc);


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
        virtual yarp::sig::Vector getJointBoundMin();


        /**
         * Returns a list containing the max value for each joint.
         */
        virtual yarp::sig::Vector getJointBoundMax();

        /**
          * Returns a list containing the max torque value for each joint.
          */
        virtual yarp::sig::Vector getJointTorqueMax();


        /**
         * Set a list containing the max torque value for each joint.
         */
        virtual bool setJointTorqueBoundMax(const yarp::sig::Vector & _tau);




        /**
         * Set a list containing the min value for each joint.
         */

        virtual bool setJointBoundMin(const yarp::sig::Vector & _q);


        /**
         * Set a list containing the max value for each joint.
         */
        virtual bool setJointBoundMax(const yarp::sig::Vector & _q);

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




        /** @name Methods to change the floating base of the robot
        *  Methods to change the floating base of the robot
        */
        //@{

        /**
         * Set the floating base link used for all the algorithms in iDynTree
         * (except for kinematicRNEA progation, for which the imu link is used).
         * @param link_index the index of the link desired to be the floating base
         * @return true if all went well, false otherwise
         */
        bool setFloatingBaseLink(const int link_index);

        /**
         * Get the floating base link used for all the algorithms in iDynTree
         * (except for kinematicRNEA progation, for which the imu link is used).
         *
         * @return the link index of the floating base link
         */
        int getFloatingBaseLink();

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
        virtual bool computeXWorld()    const;

        /**
        * Execute the kinematic phase (recursive calculation of position, velocity,
        * acceleration of each link) of the RNE algorithm.
        * @return true if succeeds, false otherwise
        */
        virtual bool kinematicRNEA();



        bool estimateDoubleSupportContactForce(int left_foot_id, int right_foot_id);

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

        virtual KDL::Frame getPositionKDL(const int link_index, bool inverse = false) const;


        /**
        * Get the 4x4 rototranslation matrix between two link frames
        * (in particular, of the second link frame expressed in the first link frame, \f$ {}^fH_s \f$))
        * @param first_link the index of the first link
        * @param second_link the index of the second link
        * @return a 4x4 rototranslation yarp::sig::Matrix
        */
        virtual yarp::sig::Matrix getPosition(const int first_link, const int second_link) const;

        virtual KDL::Frame getPositionKDL(const int first_link, const int second_link) const;

        /**
         * Get the 4x4 rototranslation matrix between two link frames
         * (in particular, of the second link frame expressed in the first link frame, \f$ {}^fH_s \f$))
         * with an offset from the first link frame. This offset is *added* to the distance vector in
         * \f$ {}^fH_s \f$.
         *
         *  @param first_link  index of the first link
         *  @param second_link index of the second link
         *  @param offset      Distance offset from first link frame.
         *
         *  @return 4x4 rototranslation KDL::Frame
         */
        virtual KDL::Frame getPositionKDL(const int first_link, const int second_link, KDL::Vector offset) const;


        /**
         *
         * \todo TODO add getVel with output reference parameters
        * Get the velocity of the specified link, expressed in the world reference frame, but using as reference point
        * the origin of the link local reference frame
        * @param link_index the index of the link
        * @param local if true, return the velocity expressed in the link local frame (default: false)
        * @return a 6x1 vector with linear velocity \f$ {}^wv_i \f$ (0:2) and angular velocity \f$ {}^w\omega_i\f$ (3:5)
        */
        virtual yarp::sig::Vector getVel(const int link_index, const bool local=false) const;


       /**
        * Get the classical acceleration of the origin of the specified link, expressed in the world reference frame
        * @param link_index the index of the link
        * @param acc a 6x1 vector with linear acc \f$ {}^ia_i \f$(0:2) and angular acceleration \f$ {}^i\dot{\omega}_i \f$ (3:5)
        * @param local if true, return the velocity expressed in the link local frame (default: false)
        * @return true if all went well, false otherwise
        *
        * \note This function returns the classical/conventional linear acceleration, not the spatial one
        */
        virtual bool getAcc(const int link_index, yarp::sig::Vector & acc, const bool local=false) const;

        /**
        * Get the classical acceleration of the origin of the specified link, expressed in the world reference frame
        * @param link_index the index of the link
        * @param local if true, return the velocity expressed in the link local frame (default: false)
        * @return a 6x1 vector with linear acc \f$ {}^ia_i \f$(0:2) and angular acceleration \f$ {}^i\dot{\omega}_i \f$ (3:5)
        *
        * \note This function returns the classical/conventional linear acceleration, not the spatial one
        */
        virtual yarp::sig::Vector getAcc(const int link_index, const bool local=false) const;

        /**
         * Get the base link force torque, calculated with the dynamic recursive newton euler loop
         *
         * @param frame_link specify the frame of reference in which express the return value (default: the dynamic base link)
         * @return a 6x1 vector with linear force \f$ {}^b f_b \f$(0:2) and angular torque\f$ {}^b\tau_b \f$ (3:5)
         */
        virtual yarp::sig::Vector getBaseForceTorque(int frame_link=DEFAULT_INDEX_VALUE);


        /**
        * Get joint torques
        * @return vector of joint torques
        */
        virtual yarp::sig::Vector getTorques() const;

        /**
         * Get the ForceTorque transmitted through joint joint_index.
         * This method returns the force applied by the child link
         * on the parent link of the joint, expressed in the child Pluker coordinate frame.
         *
         * @param joint_index joint of the requested forcetorque
         * @param frame_link  specify the plucker frame of reference in which express the return value (default: the child link)
         * \note the definition of parent and child link is the one stored in the joint, so the "original one" of the model,
         *       not the one modified by changing the base
         *
         * @return a 6x1 vector with linear force \f$ {}^b f_b \f$(0:2) and angular torque\f$ {}^b\tau_b \f$ (3:5)
         */
        yarp::sig::Vector getJointForceTorque(int joint_index, int frame_link) const;

        /**
         * Get the external ForceTorque (Wrench) acting on a given link.
         * @param link_index the index of the link for which we get the external wrench
         * @param origin_frame_index the returned wrench will be expressed at the origin of origin_frame_index
         * @param orientation_frame_index the returned wrench will be expressed with the orientation of orientation_frame_index
         * @return a 6x1 vector with linear force \f$ {}^b f_b \f$(0:2) and angular torque\f$ {}^b\tau_b \f$ (3:5)
         */
        KDL::Wrench getExternalForceTorqueKDL(int link_index, int origin_frame_index, int orientation_frame_index);


        /**
         * Get the external ForceTorque (Wrench) acting on a given link.
         * @param link_index the index of the link for which we get the external wrench
         * @param origin_frame_index the returned wrench will be expressed at the origin of origin_frame_index
         * @param orientation_frame_index the returned wrench will be expressed with the orientation of orientation_frame_index
         * @return a 6x1 vector with linear force \f$ {}^b f_b \f$(0:2) and angular torque\f$ {}^b\tau_b \f$ (3:5)
         */
        yarp::sig::Vector getExternalForceTorque(int link_index, int origin_frame_index, int orientation_frame_index);


        //@}
        /** @name Methods related to Jacobian calculations
        *
        *
        */
        //@{


        virtual bool getJacobianKDL(const int link_index, KDL::Jacobian & jac, bool local=false);


        /**
        *
        * For a floating base structure, outpus a 6x(nrOfDOFs+6) yarp::sig::Matrix \f$ {}^w J_i \f$ such
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

        /**
        * Get the 6+getNrOfDOFs() yarp::sig::Vector, characterizing the floating base acceleration of the tree
        * @return a vector where the 0:5 elements are the one of the dynamic base expressed in the world frame (the same that are obtained calling
        *         getAcc(dynamic_base_index), while the 6:6+getNrOfDOFs()-1 elements are the joint accelerations
        */
        virtual yarp::sig::Vector getD2Q_fb() const;


        virtual bool getRelativeJacobianKDL(const int jacobian_distal_link, const int jacobian_base_link, KDL::Jacobian & jac, bool global=false);


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

        KDL::Vector getCOMKDL(const int link_index = -1) const;


        /**
        * Get Center of Mass of the robot expressed in the world frame
        * @param link_index if indicated, express the returned center of mass in the link reference frame
        * @return Center of Mass vector
        *
        * \todo prepare a suitable interface for specifing both an arbitrary part and a arbitrary frame of expression
        */
        virtual yarp::sig::Vector getCOM(const int link_index = -1) const;

        virtual bool getCOMJacobianKDL(KDL::Jacobian & jac);

        /**
        * Get Center of Mass Jacobian of the specified part (if no part
        * is specified, get the Jacobian of the COM of all the tree) expressed in
        * the world frame
        * @param jac the output jacobiam matrix
        * @return true if succeeds, false otherwise
        */
        virtual bool getCOMJacobian(yarp::sig::Matrix & jac);


        virtual bool getCOMJacobianKDL(KDL::Jacobian & jac, KDL::CoDyCo::MomentumJacobian & momentum_jac);

        /**
         * Temporary function, do not use.
         *
         */
        virtual bool getCOMJacobian(yarp::sig::Matrix & jac, yarp::sig::Matrix & momentum_jac);

        /**
         * Temporary function, do not use.
         *
         */
        virtual bool getCentroidalMomentumJacobian(yarp::sig::Matrix & jac);


        /**
        * Get Velocity of the Center of Mass of the robot expressed in the world frame
        * @return velocity of Center of Mass vector
        */
        yarp::sig::Vector getVelCOM();

       /**
        * Get the acceleration (3d) of the Center of Mass of the
        * robot expressed in the world frame
        * @return acceleration of Center of Mass vector
        */
        yarp::sig::Vector getAccCOM();

       /**
        * Get the acceleration (3d) of the Center of Mass of the
        * robot expressed in the world frame
        * @param com_acceleration the output vector for acceleration of Center of Mass vector
        * @return true if all went well, false otherwise
        */
        bool getAccCOM(yarp::sig::Vector & com_acceleration);

        yarp::sig::Vector getMomentum();
        yarp::sig::Vector getCentroidalMomentum();


        //@}


        /** @name Methods related to mass matrix computations
        *
        *
        */
        //@{

        /**
         * Return the (6+n_dof)x(6+n_dof) floating base mass matrix \f[ \mathbf{M} \f], such that the kinematic energy
         * of the system is given by:
         * \f[
         *  \dot{\mathbf{q}}^\top \mathbf{M} \dot{\mathbf{q}}
         * \f]
         * where \f[ \dot{\mathbf{q}} \in \mathbb{R}^{6+n} \f] is defined by abuse of notation as the concatenation of
         * \f[ {}^w \mathbf{v} \in \mathbb{R}^6 \f] (the floating base origin velocity expressed in world frame) and
         * \f[ {}^w \dot{\boldsymbol\theta} \in \mathbb{R}^n \f] (the joint velocity vector)
         *
         * @param fb_mass_matrix the yarp::sig::Matrix used to return the mass matrix, it will be resized if not \f[ {}^w \mathbf{v} \in \mathbb{R}^6 \f]
         * @return true if all went well, false otherwise
         */
        bool getFloatingBaseMassMatrix(yarp::sig::Matrix & fb_mass_matrix);


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

        /**
         * @name Methods related to debug
        *
        *
        */
        //@{
        KDL::Tree getKDLTree() { return undirected_tree.getTree(); }

        KDL::CoDyCo::UndirectedTree getKDLUndirectedTree() { return undirected_tree; }


        //@}

};

}//end namespace

}

#endif //end IDYNTREE_H
