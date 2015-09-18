/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 */

#include <iCub/iDynTree/DynTree.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <iCub/iDynTree/yarp_iDynTree.h>

//Loops from KDL_CoDyCo
#include <kdl_codyco/position_loops.hpp>
#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/regressor_loops.hpp>
#include <kdl_codyco/jacobian_loops.hpp>
#include <kdl_codyco/com_loops.hpp>
#include <kdl_codyco/crba_loops.hpp>
#include <kdl_codyco/utils.hpp>
#include <kdl_codyco/regressor_utils.hpp>
#include <kdl_codyco/generalizedjntpositions.hpp>

#include "iDynTree/Sensors/SixAxisFTSensor.hpp"
#include "kdl_codyco/KDLConversions.h"
#include "iDynTree/Core/Transform.h"
#include "iDynTree/Core/Wrench.h"
#include "iDynTree/Core/SpatialForceVector.h"

#include <kdl_codyco/regressor_utils.hpp>
#include <kdl_codyco/regressors/dirl_utils.hpp>

#include <iDynTree/ModelIO/impl/urdf_import.hpp>

#include <kdl/frames_io.hpp>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <yarp/os/LogStream.h>

#include <vector>

using namespace yarp::sig;
using namespace yarp::math;

/**
 *
 * \todo refactor iDynTreeContact and solve dynamic allocation of regressor matrix for contacts
 *
 */

namespace iCub {
namespace iDynTree {


DynTree::DynTree(): configured(false)
{
}

DynTree::DynTree(const KDL::Tree & _tree,
                 KDL::CoDyCo::TreeSerialization serialization):
                 configured(false),
                 undirected_tree(_tree,serialization)
{
    std::string imu_link_name="";
    std::vector<std::string> joint_sensor_names;
    constructor(_tree,joint_sensor_names,imu_link_name,serialization);
}

DynTree::DynTree(const KDL::Tree & _tree,
                   const std::vector<std::string> & joint_sensor_names,
                   const std::string & imu_link_name,
                   KDL::CoDyCo::TreeSerialization serialization
                   ):  configured(false), undirected_tree(_tree,serialization)
{
    constructor(_tree,joint_sensor_names,imu_link_name,serialization);
}

DynTree::DynTree(const std::string urdf_file,
                 const std::vector<std::string> & joint_sensor_names,
                 const std::string & imu_link_name,
                 KDL::CoDyCo::TreeSerialization  serialization): configured(false)
{
    KDL::Tree my_tree;
    if (!::iDynTree::treeFromUrdfFile(urdf_file,my_tree))
    {
        std::cerr << "DynTree constructor: Could not generate robot model from file " << urdf_file << "  and extract kdl tree" << std::endl; assert(false);
    }
    constructor(my_tree,joint_sensor_names,imu_link_name,serialization);


    //Loading joint limits from URDF
    yarp::sig::Vector yarpJointMinLimit(NrOfDOFs), yarpJointMaxLimit(NrOfDOFs);
    DynTree::loadJointLimitsFromURDFFile(urdf_file, undirected_tree, yarpJointMinLimit, yarpJointMaxLimit);

    this->setJointBoundMin(yarpJointMinLimit);
    this->setJointBoundMax(yarpJointMaxLimit);
}

bool DynTree::loadURDFModel(const std::string & urdf_file)
{
    KDL::Tree my_tree;
    if (!::iDynTree::treeFromUrdfFile(urdf_file,my_tree))
    {
        std::cerr << "DynTree loadURDFModel: Could not generate robot model from file " << urdf_file << "  and extract kdl tree" << std::endl;
        return false;
    }

    std::vector<std::string> no_ft_sensors_names;
    std::string no_imu = "";
    constructor(my_tree,no_ft_sensors_names,no_imu);

    bool ok = this->configured;

    //Loading joint limits from URDF
    yarp::sig::Vector yarpJointMinLimit(NrOfDOFs), yarpJointMaxLimit(NrOfDOFs);
    ok = ok && DynTree::loadJointLimitsFromURDFFile(urdf_file, undirected_tree, yarpJointMinLimit, yarpJointMaxLimit);

    ok = ok && this->setJointBoundMin(yarpJointMinLimit);
    ok = ok && this->setJointBoundMax(yarpJointMaxLimit);

    if( !ok )
    {
        std::cerr << "DynTree loadURDFModel: Could not generate robot model from file " << urdf_file << std::endl;
        std::cerr << "If you believe iDynTree is failing to parse a valid URDF file, please open a bug at " << std::endl;
        std::cerr << "https://github.com/robotology-playground/idyntree/issues/new" << std::endl;
    }

    return ok;
}

void DynTree::constructor(const KDL::Tree & _tree,
                          const std::vector<std::string> & joint_sensor_names,
                          const std::string & imu_link_name,
                          KDL::CoDyCo::TreeSerialization serialization
)
{
    int ret;

    undirected_tree = KDL::CoDyCo::UndirectedTree(_tree,serialization);

    #ifndef NDEBUG
    //std::cout << "DynTree serialization " << undirected_tree.getSerialization().toString() << std::endl;
    //std::cout << "DynTree partition: " << partition.toString() << std::endl;
    #endif
    //Setting useful constants
    NrOfDOFs = _tree.getNrOfJoints();
    NrOfLinks = undirected_tree.getNrOfLinks();
    NrOfFTSensors = joint_sensor_names.size();
    m_joint_sensor_names = joint_sensor_names;
    NrOfDynamicSubGraphs = NrOfFTSensors + 1;

    assert(undirected_tree.getNrOfDOFs() == NrOfDOFs);
    //Remve assertion for robot without a proper fixed base
    //assert((int)undirected_tree.getNrOfLinks() == NrOfLinks);

    world_base_frame = KDL::Frame::Identity();

    q = KDL::JntArray(NrOfDOFs);
    SetToZero(q);
    is_X_dynamic_base_updated = false;
    is_X_world_updated = false;

    dq = KDL::JntArray(NrOfDOFs);
    SetToZero(dq);
    ddq = KDL::JntArray(NrOfDOFs);
    SetToZero(ddq);

    torques = KDL::JntArray(NrOfDOFs);

    q_jnt_max = KDL::JntArray(NrOfDOFs);
    q_jnt_min = KDL::JntArray(NrOfDOFs);
    tau_max = KDL::JntArray(NrOfDOFs);

    constrained = std::vector<bool>(NrOfDOFs,false);
    constrained_count = 0;

    kinematic_traversal = KDL::CoDyCo::Traversal();
    dynamic_traversal = KDL::CoDyCo::Traversal();

    //measured_wrenches =  std::vector< KDL::Wrench >(NrOfFTSensors);
    sensor_measures.setNrOfSensors(::iDynTree::SIX_AXIS_FORCE_TORQUE,NrOfFTSensors);

    v = std::vector<KDL::Twist>(NrOfLinks);
    a = std::vector<KDL::Twist>(NrOfLinks);

    f = std::vector<KDL::Wrench>(NrOfLinks); /**< wrench trasmitted by a link to the predecessor (note that predecessor definition depends on the selected dynamic base */
    f_gi = std::vector<KDL::Wrench>(NrOfLinks);

    f_ext = std::vector<KDL::Wrench>(NrOfLinks,KDL::Wrench::Zero());

    //Create default kinematic traversal (if imu_names is wrong, creating the default one)
    if( imu_link_name != "" ) {
        ret = undirected_tree.compute_traversal(kinematic_traversal,imu_link_name);
    } else {
        ret = undirected_tree.compute_traversal(kinematic_traversal);
    }

    if( ret < 0 ) { std::cerr << "iDynTree constructor: imu_link_name not found" << std::endl; }
    assert(ret == 0);

    //Create default dynamics traversal (the dynamical base is the default one of the KDL::Tree)
    //Note that the selected dynamical base affect only the "classical" use case, when unkown external
    //wrenches are not estimated and are assume acting on the base.
    //If the external forces are properly estimated, the base link for dynamic loop should not
    //affect the results (i.e.
    ret = undirected_tree.compute_traversal(dynamic_traversal);
    assert(ret == 0);

    three_zeros.resize(3);
    three_zeros.zero();

    assert(ret == 0);
    configured = true;
}



DynTree::~DynTree() { }

double DynTree::setAng(const double q_in, const int i)
{
    is_X_dynamic_base_updated = false;
    is_X_world_updated = false;

    if (constrained[i]) {
        q(i) = (q_in<q_jnt_min(i)) ? q_jnt_min(i) : ((q_in>q_jnt_max(i)) ? q_jnt_max(i) : q_in);
    } else {
        q(i) = q_in;
    }
    return q(i);
}



/*
iDynTreeLinkAndFrame DynTree::getiDynTreeLinkFrameFromSkinDynLibID(int body_part, int link)
{
    // std::cout << "getLinkFromSkinDynLibID" << body_part << " " << link << std::endl;
    skinDynLibLinkID sdl_id;
    sdl_id.body_part = body_part;
    sdl_id.local_link_index = link;
    for( std::map<skinDynLibLinkID,iDynTreeLinkFrame>::iterator it = skinDynLibLinkMap.begin();
         it != skinDynLibLinkMap.end(); it++ )
    {
        if( it->first.body_part == body_part &&
            it->first.local_link_index == link)
        {
            return it->second;
        }
    }
    iDynTreeLinkAndFrame error_id;
    error_id.link_index = -1;
    error_id.frame_index = -1;
    return error_id;
}*/


//====================================
//
//      Set/Get methods
//
//====================================
bool DynTree::setWorldBasePoseKDL(const KDL::Frame & H_w_b)
{
    world_base_frame = H_w_b;
	return true;
}

bool DynTree::setWorldBasePose(const yarp::sig::Matrix & H_w_b)
{
    if ((H_w_b.rows()==4) && (H_w_b.cols()==4))
    {
        return YarptoKDL(H_w_b,world_base_frame);
    }
    else
    {
        if (verbose)
            std::cerr << "DynTree::setWorldBasePose: Attempt to reference a wrong matrix H_w_p (not 4x4)" << std::endl;

        return false;
    }
}

KDL::Frame DynTree::getWorldBasePoseKDL()
{
    return world_base_frame;
}


yarp::sig::Matrix DynTree::getWorldBasePose()
{
    KDLtoYarp_position(world_base_frame,_H_w_b);
    return _H_w_b;
}


yarp::sig::Vector DynTree::getAng() const
{
    yarp::sig::Vector ret;
    KDLtoYarp(q,ret);
    return ret;
}

bool DynTree::getAngKDL(KDL::JntArray & _q) const
{
    _q = q;
    return true;
}


bool DynTree::setAngKDL(const KDL::JntArray & _q)
{
    is_X_dynamic_base_updated = false;
    is_X_world_updated = false;


        //No part specified
        if( (int)_q.rows() != NrOfDOFs )
        {
            std::cerr << "setAng: Input vector has a wrong number of elements" << std::endl;
            return false;
        }
        if( constrained_count == 0 ) {
            //if all are not constrained, use a quicker way to copy
            q = _q;
        } else {
            for(int i =0; i < NrOfDOFs; i++ ){
                setAng(_q(i),i);
            }
        }

    return true;
}

yarp::sig::Vector DynTree::setAng(const yarp::sig::Vector & _q)
{
    is_X_dynamic_base_updated = false;
    is_X_world_updated = false;


    yarp::sig::Vector ret_q = _q;

        if( (int)_q.size() != NrOfDOFs ) { std::cerr << "setAng: Input vector has a wrong number of elements" << std::endl; return yarp::sig::Vector(0); }
        if( constrained_count == 0 ) {
            //if all are not constrained, use a quicker way to copy
            YarptoKDL(_q,q);
        } else {
            for(int i =0; i < NrOfDOFs; i++ ){
                ret_q[i] = setAng(_q[i],i);
            }
        }


    return ret_q;
}

yarp::sig::Vector DynTree::setDAng(const yarp::sig::Vector & _q)
{

        if( (int)_q.size() != NrOfDOFs  ) { std::cerr << "setDAng: Input vector has a wrong number of elements" << std::endl; return yarp::sig::Vector(0); }
        YarptoKDL(_q,dq);

    return _q;
}

yarp::sig::Vector DynTree::getDAng() const
{
    yarp::sig::Vector ret;
    KDLtoYarp(dq,ret);
    return ret;

}

bool DynTree::getDAngKDL(KDL::JntArray & _dq) const
{
    _dq = dq;
    return true;
}

bool DynTree::setDAngKDL(const KDL::JntArray & _dq)
{
    if( _dq.rows() != dq.rows() )
    {
        return false;
    }

    dq = _dq;

    return true;
}


yarp::sig::Vector DynTree::getDQ_fb() const
{
    /**
     *
     * \todo checking it is possible to return something which have sense
     */
    return cat(getVel(dynamic_traversal.getBaseLink()->getLinkIndex()),getDAng());
}

yarp::sig::Vector DynTree::getD2Q_fb() const
{
    /**
     *
     * \todo checking it is possible to return something which have sense
     */
    return cat(getAcc(dynamic_traversal.getBaseLink()->getLinkIndex()),getD2Ang());
}

yarp::sig::Vector DynTree::setD2Ang(const yarp::sig::Vector & _q)
{
    if( (int)_q.size() != NrOfDOFs  ) { std::cerr << "setD2Ang: Input vector has a wrong number of elements" << std::endl; return yarp::sig::Vector(0); }
        YarptoKDL(_q,ddq);
    return _q;
}

yarp::sig::Vector DynTree::getD2Ang() const
{
    yarp::sig::Vector ret;

    KDLtoYarp(ddq,ret);

    return ret;
}

bool DynTree::getD2AngKDL(KDL::JntArray & _d2q) const
{
    _d2q = ddq;
    return true;
}

bool DynTree::setD2AngKDL(const KDL::JntArray & _d2q)
{
    if( _d2q.rows() != ddq.rows() )
    {
        return false;
    }

    ddq = _d2q;
    return true;
}

bool DynTree::setGravity(const yarp::sig::Vector &gravity)
{
    return setInertialMeasureAndLinearVelocity(three_zeros,three_zeros,gravity,three_zeros);
}

bool DynTree::setInertialMeasureAndLinearVelocity(const yarp::sig::Vector &dp0, const yarp::sig::Vector &w0, const yarp::sig::Vector &ddp0, const yarp::sig::Vector &dw0)
{
    KDL::Twist imu_classical_acceleration;
    KDL::Vector w0_kdl, dw0_kdl, dp0_kdl, ddp0_kdl;
    YarptoKDL(dp0,dp0_kdl);
    YarptoKDL(w0,w0_kdl);
    YarptoKDL(dw0,dw0_kdl);
    YarptoKDL(ddp0,ddp0_kdl);
    imu_velocity.vel = dp0_kdl;
    imu_velocity.rot = w0_kdl;
    imu_classical_acceleration.vel = ddp0_kdl;
    imu_classical_acceleration.rot = dw0_kdl;
    KDL::CoDyCo::conventionalToSpatialAcceleration(imu_classical_acceleration,imu_velocity,imu_acceleration);
    return true;
}

bool DynTree::setInertialMeasure(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0)
{
    return setInertialMeasureAndLinearVelocity(three_zeros,w0,ddp0,dw0);
}

bool DynTree::setKinematicBaseVelAcc(const yarp::sig::Vector &base_vel, const yarp::sig::Vector &base_classical_acc)
{
    //std::cout << "base vel " << base_vel.toString() << std::endl;
    KDL::Twist base_vel_kdl, base_classical_acc_kdl, base_spatial_acc_kdl;
    YarptoKDL(base_vel,base_vel_kdl);
    //std::cout << "base_vel_kdl " << base_vel_kdl << std::endl;
    YarptoKDL(base_classical_acc,base_classical_acc_kdl);
    KDL::CoDyCo::conventionalToSpatialAcceleration(base_classical_acc_kdl,base_vel_kdl,base_spatial_acc_kdl);
    computePositions();
    //std::cout << "world_base_frame " << world_base_frame << std::endl;
    //std::cout << "X_dynamic_base_kinematic_base " << X_dynamic_base[kinematic_traversal.getBaseLink()->getLinkIndex()] << std::endl;
    imu_velocity = (world_base_frame*X_dynamic_base[kinematic_traversal.getBaseLink()->getLinkIndex()]).M.Inverse(base_vel_kdl);
    //std::cout << "imu_velocity " << imu_velocity << std::endl;
    imu_acceleration = (world_base_frame*X_dynamic_base[kinematic_traversal.getBaseLink()->getLinkIndex()]).M.Inverse(base_spatial_acc_kdl);
    //std::cout << "imu_velocity " << imu_acceleration << std::endl;


    return true;
}

bool DynTree::getInertialMeasure(yarp::sig::Vector &w0, yarp::sig::Vector &dw0, yarp::sig::Vector &ddp0) const
{
    //should care about returning the 3d acceleration instead of the spatial one? yes
    //assuming the base linear velocity as 0, they are the same
    KDL::Twist imu_classical_acceleration;
    KDL::CoDyCo::spatialToConventionalAcceleration(imu_acceleration,imu_velocity,imu_classical_acceleration);
    KDLtoYarp(imu_velocity.rot,w0);
    KDLtoYarp(imu_classical_acceleration.vel,ddp0);
    KDLtoYarp(imu_classical_acceleration.rot,dw0);
    return true;
}

bool DynTree::setSensorMeasurement(const int sensor_index, const yarp::sig::Vector &ftm)
{
    if( sensor_index < 0 ||
        sensor_index > (int) NrOfFTSensors )
    {
        return false;
    }

    if( ftm.size() != 6 )
    {
        return false;
    }

    //::iDynTree::Wrench measured_wrench;
    //YarptoKDL(ftm.subVector(0,2),measured_wrench.force);
    //YarptoKDL(ftm.subVector(3,5),measured_wrench.torque);

    ::iDynTree::Wrench measured_wrench;
    YarptoiDynTree(ftm,measured_wrench);
    //YarptoiDynTree(ftm.subVector(3,5),measured_wrench.torque);

    bool ret = sensor_measures.setMeasurement(::iDynTree::SIX_AXIS_FORCE_TORQUE,sensor_index,measured_wrench);

    are_contact_estimated = false;

    return ret;
}

bool DynTree::getSensorMeasurement(const int sensor_index, yarp::sig::Vector &ftm) const
{
    //\todo avoid unnecessary memory allocation
    //yarp::sig::Vector force_yarp(3);
    //yarp::sig::Vector torque_yarp(3);
    yarp::sig::Vector yarpWrench(6);
    if( sensor_index < 0 ||
        sensor_index > (int)NrOfFTSensors )
    {
        return false;
    }

    if( ftm.size() != 6 )
    {
        ftm.resize(6);
    }

    ::iDynTree::Wrench measured_wrench;

    bool ok = sensor_measures.getMeasurement(::iDynTree::SIX_AXIS_FORCE_TORQUE,sensor_index,measured_wrench);

    assert(ok);

    //KDLtoYarp(measured_wrench.force,force_yarp);
    //KDLtoYarp(measured_wrench.torque,torque_yarp);

    iDynTreetoYarp(measured_wrench,yarpWrench);
    //ftm.setSubvector(0,force_yarp);
    //ftm.setSubvector(3,torque_yarp);
    ftm.setSubvector(0,yarpWrench);
    return true;
}

yarp::sig::Vector DynTree::getJointBoundMin()
{
    yarp::sig::Vector ret;

    KDLtoYarp(q_jnt_min,ret);


    return ret;
}

yarp::sig::Vector DynTree::getJointTorqueMax()
{
    yarp::sig::Vector ret;

    KDLtoYarp(tau_max,ret);

    return ret;
}

yarp::sig::Vector DynTree::getJointBoundMax()
{
    yarp::sig::Vector ret;

    KDLtoYarp(q_jnt_max,ret);

    return ret;
}

bool DynTree::setJointBoundMin(const yarp::sig::Vector & _q)
{
    if( (int)_q.size() != NrOfDOFs  ) { std::cerr << "setJointBoundMin error: input vector has size " << _q.size() <<  " while should have size " << NrOfDOFs << std::endl; return false; }
    YarptoKDL(_q,q_jnt_min);

    return true;
}

bool DynTree::setJointBoundMax(const yarp::sig::Vector & _q)
{
        if( (int)_q.size() != NrOfDOFs  ) { std::cerr << "setJointBoundMax error: input vector has size " << _q.size() <<  " while should have size " << NrOfDOFs << std::endl; return false; }
        YarptoKDL(_q,q_jnt_max);

    return true;
}

bool DynTree::setJointTorqueBoundMax(const yarp::sig::Vector & _tau)
{
        if( (int)_tau.size() != NrOfDOFs  ) { std::cerr << "setTorqueJointBoundMax error: input vector has size " << _tau.size() <<  " while should have size " << NrOfDOFs << std::endl; return false; }
        YarptoKDL(_tau,tau_max);

    return true;
}

void DynTree::setAllConstraints(bool _constrained)
{
    if( _constrained ) {
        //all joints are now not constrained
        for(size_t i=0; i < constrained.size(); i++ ) constrained[i] = false;
        constrained_count = 0;
    } else {
        //all joints are now constrained
        for(size_t i=0; i < constrained.size(); i++ ) constrained[i] = true;
        constrained_count = constrained.size();
    }
}

void DynTree::setConstraint(unsigned int i, bool _constrained)
{
    //If a joint is constrained, add 1 to the number of constrained joints
    if( !constrained[i] && _constrained ) constrained_count++;
    //If a joint is liberated from its constraint, subtract 1 from the number of constrained joints
    if( constrained[i] && !_constrained ) constrained_count--;

    constrained[i] = _constrained;
}

bool DynTree::getConstraint(unsigned int i) { return constrained[i]; }

bool DynTree::setFloatingBaseLink(const int link_index)
{
    // \todo add a method to invalidate all the buffers
    int old_flt_base_link = getFloatingBaseLink();
    is_X_dynamic_base_updated = false;
    is_X_world_updated = false;
    if( undirected_tree.compute_traversal(dynamic_traversal,link_index) == 0)
    {
        return true;
    } else {
        int check = undirected_tree.compute_traversal(dynamic_traversal,old_flt_base_link);
        assert(check==0);
        return false;
    }
}

int DynTree::getFloatingBaseLink()
{
    return dynamic_traversal.getBaseLink()->getLinkIndex();
}


yarp::sig::Matrix DynTree::getPosition(const int link_index,bool inverse) const
{
    return KDLtoYarp_position(getPositionKDL(link_index,inverse));
}

KDL::Frame DynTree::getPositionKDL(const int link_index,bool inverse) const
{
    if( link_index < 0 || link_index >= (int)undirected_tree.getNrOfLinks() ) { std::cerr << "DynTree::getPosition: link index " << link_index <<  " out of bounds" << std::endl; return error_frame; }
    computePositions();
    if( !inverse ) {
        return (world_base_frame*X_dynamic_base[link_index]);
    } else {
        return ((world_base_frame*X_dynamic_base[link_index]).Inverse());
    }
}

yarp::sig::Matrix DynTree::getPosition(const int first_link, const int second_link) const
{
    return KDLtoYarp_position(getPositionKDL(first_link,second_link));
}

KDL::Frame DynTree::getPositionKDL(const int first_link, const int second_link) const
{
   if( first_link < 0
       || first_link >= (int)undirected_tree.getNrOfLinks() )
   {
       std::cerr << "DynTree::getPosition: link index " << first_link <<  " out of bounds" << std::endl;
       return error_frame;
   }

   if( second_link < 0
       || second_link >= (int)undirected_tree.getNrOfLinks() )
   {
       std::cerr << "DynTree::getPosition: link index " << second_link <<  " out of bounds" << std::endl;
       return error_frame;
   }
   computePositions();
   return (X_dynamic_base[first_link].Inverse()*X_dynamic_base[second_link]);
}

yarp::sig::Vector DynTree::getVel(const int link_index, const bool local) const
{
    if( link_index < 0 || link_index >= (int)undirected_tree.getNrOfLinks() ) {
        std::cerr << "DynTree::getVel: link index " << link_index <<  " out of bounds" << std::endl;
        return yarp::sig::Vector(0);
    }
    yarp::sig::Vector ret(6), lin_vel(3), ang_vel(3);
    KDL::Twist return_twist;

    if( !local ) {
        computePositions();
        return_twist = (world_base_frame*X_dynamic_base[link_index]).M*(v[link_index]);
    } else {
        return_twist = v[link_index];
    }

    KDLtoYarp(return_twist.vel,lin_vel);
    KDLtoYarp(return_twist.rot,ang_vel);
    ret.setSubvector(0,lin_vel);
    ret.setSubvector(3,ang_vel);


    return ret;
}

yarp::sig::Vector DynTree::getAcc(const int link_index, const bool local) const
{
    yarp::sig::Vector acc(6);
    if( getAcc(link_index,acc,local) ) {
        return acc;
    } else {
        return yarp::sig::Vector(0);
    }
}

bool DynTree::getAcc(const int link_index, yarp::sig::Vector & acc, const bool local) const
{
    if( link_index < 0 || link_index >= (int)undirected_tree.getNrOfLinks() ) {
        std::cerr << "DynTree::getAcc: link index " << link_index <<  " out of bounds" << std::endl;
        return false;
    }
    /*
    yarp::sig::Vector ret(6), classical_lin_acc(3), ang_acc(3);
    KDLtoYarp(a[link_index].vel+v[link_index].rot*v[link_index].vel,classical_lin_acc);
    KDLtoYarp(a[link_index].rot,ang_acc);
    ret.setSubvector(0,classical_lin_acc);
    ret.setSubvector(3,ang_acc);
    */
    KDL::Twist classical_acc, return_acc;
    KDL::CoDyCo::spatialToConventionalAcceleration(a[link_index],v[link_index],classical_acc);

    if( !local ) {
        computePositions();
        return_acc = (world_base_frame*X_dynamic_base[link_index]).M*(classical_acc);
    } else {
        return_acc = classical_acc;
    }

    return KDLtoYarp(return_acc,acc);
}

yarp::sig::Vector DynTree::getBaseForceTorque(int frame_link)
{
    yarp::sig::Vector ret(6), ret_force(3), ret_torque(3);
    KDL::Wrench ret_kdl;
    if( frame_link == DEFAULT_INDEX_VALUE ) { frame_link = dynamic_traversal.getBaseLink()->getLinkIndex(); }

    if( frame_link != WORLD_FRAME && (frame_link < 0 || frame_link >= getNrOfLinks()) ) { std::cerr << "DynTree::getBaseFroceTorque: link index " << frame_link <<  " out of bounds" << std::endl; return yarp::sig::Vector(0); }
    if( frame_link == dynamic_traversal.getBaseLink()->getLinkIndex() )
    {
        ret_kdl = base_residual_f;
    } else if( frame_link == WORLD_FRAME ) {
        ret_kdl = world_base_frame.M*base_residual_f;
    } else {
        computePositions();
        ret_kdl = X_dynamic_base[frame_link].M.Inverse(base_residual_f);
    }
    KDLtoYarp(ret_kdl.force,ret_force);
    KDLtoYarp(ret_kdl.torque,ret_torque);
    ret.setSubvector(0,ret_force);
    ret.setSubvector(3,ret_torque);
    return ret;
}

yarp::sig::Vector DynTree::getTorques() const
{
    #ifndef NDEBUG
    //std::cout << "DynTree::getTorques(" << part_name << ")" << std::endl;
    #endif

        yarp::sig::Vector ret(NrOfDOFs);
        KDLtoYarp(torques,ret);
        return ret;

}

yarp::sig::Vector DynTree::getJointForceTorque(int joint_index, int frame_link) const
{
    if( joint_index < 0 || joint_index >= f.size() )
    {
        std::cerr << "getJointForceTorque: joint_index " << joint_index << " out of bounds " << std::endl;
        return yarp::sig::Vector(0);
    }

    bool is_child_to_parent = false;

    //Get the two links connected to the joint
    int link1 = undirected_tree.getJunction(joint_index)->getParentLink()->getLinkIndex();
    int link2 = undirected_tree.getJunction(joint_index)->getChildLink()->getLinkIndex();
    int parent_link, child_link;

    //Get the child link (given the traversal)
    if( link1 == dynamic_traversal.getParentLink(link2)->getLinkIndex() )
    {
        parent_link = link1;
        child_link = link2;
        is_child_to_parent = true;
    }
    else
    {
        if(link2 != dynamic_traversal.getParentLink(link1)->getLinkIndex() )
        {
            std::cerr << "getJointForceTorque: inconsistent dynamic_traversal object " << std::endl;
            return yarp::sig::Vector(0);
        }
        parent_link = link2;
        child_link = link1;
        is_child_to_parent = false;
    }


    KDL::Wrench ret_kdl;
    if( frame_link == DEFAULT_INDEX_VALUE )
    {
        frame_link = child_link;
    }

    if( frame_link != WORLD_FRAME && (frame_link < 0 || frame_link >= getNrOfLinks()) )
    {
        std::cerr << "DynTree::getBaseFroceTorque: link index " << frame_link
                  <<  " out of bounds" << std::endl; return yarp::sig::Vector(0);
    }

    if( frame_link == child_link )
    {
        ret_kdl = f[child_link];
    } else if( frame_link == WORLD_FRAME ) {
        computePositions();
        ret_kdl = (world_base_frame*X_dynamic_base[child_link])*f[child_link];
    } else {
        computePositions();
        ret_kdl = X_dynamic_base[frame_link].Inverse()*X_dynamic_base[child_link]*f[child_link];
    }

    if(!is_child_to_parent)
    {
        ret_kdl = -ret_kdl;
    }

    yarp::sig::Vector ret(6), force(3), torque(3);
    KDLtoYarp(ret_kdl.force,force);
    KDLtoYarp(ret_kdl.torque,torque);
    ret.setSubvector(0,force);
    ret.setSubvector(3,torque);
    return ret;
}

KDL::Wrench DynTree::getExternalForceTorqueKDL(int link_index,
                                               int origin_frame_index,
                                               int orientation_frame_index)
{
    if(     link_index < 0 || link_index > this->getNrOfLinks()
        ||  origin_frame_index < 0 || origin_frame_index > this->getNrOfFrames()
        ||  orientation_frame_index < 0 || orientation_frame_index > this->getNrOfFrames() )
    {
       yError() << "getExternalForceTorqueKDL: error in input parameters";
       return KDL::Wrench::Zero();
    }

    KDL::Frame origin_frame_H_link = this->getPositionKDL(origin_frame_index,link_index);
    KDL::Wrench f_ext_translated = KDL::Frame(origin_frame_H_link.p)*f_ext[link_index];

    KDL::Frame orientation_frame_H_link = this->getPositionKDL(orientation_frame_index,link_index);
    KDL::Wrench f_ext_translated_and_rotated = orientation_frame_H_link.M*f_ext_translated;

    return f_ext_translated_and_rotated;
}

yarp::sig::Vector DynTree::getExternalForceTorque(int link_index,
                                                  int origin_frame_index,
                                                  int orientation_frame_index)
{
    yarp::sig::Vector ret(6);

    KDLtoYarp(getExternalForceTorqueKDL(link_index,origin_frame_index,orientation_frame_index),ret);

    return ret;
}


//====================================
//
//      Computation methods
//
//====================================
bool DynTree::computePositions() const
{
    if( !is_X_dynamic_base_updated ) {
        sixteen_double_zero.resize(16);
        sixteen_double_zero.zero();
        error_frame.Make4x4(sixteen_double_zero.data());

        if(X_dynamic_base.size() != undirected_tree.getNrOfLinks()) { X_dynamic_base.resize(undirected_tree.getNrOfLinks()); }
        if( getFramesLoop(undirected_tree,q,dynamic_traversal,X_dynamic_base) == 0 ) {
            is_X_dynamic_base_updated = true;
            is_X_world_updated = false;
            return true;
        }
        //else
        return false;
    } else {
        return true;
    }
}

bool DynTree::computeXWorld() const
{
    computePositions();
    if( !is_X_world_updated )
    {
        if(X_world.size() != undirected_tree.getNrOfLinks())
        {
            X_world.resize(undirected_tree.getNrOfLinks());
        }

        for(int i =0; i < X_world.size(); i++ )
        {
            X_world[i] = world_base_frame*X_dynamic_base[i];
        }
    }
    return true;
}

bool DynTree::kinematicRNEA()
{
    int ret;

    //ret = rneaKinematicLoop(undirected_tree,q,dq,ddq,kinematic_traversal,imu_velocity,imu_acceleration,v,a);
    ret = rneaKinematicLoop(undirected_tree,q,dq,ddq,kinematic_traversal,imu_velocity,imu_acceleration,v,a,f_gi);

    for( int i = 0; i < f_ext.size(); i++ )
    {
        SetToZero(f_ext[i]);
    }

    are_contact_estimated = false;

    if( ret < 0 ) return false;
    //else
    return true;
}

void pseudoInverse(const Eigen::Matrix<double, 6, 6+6>& A,
                                                        double tol,
                                                         Eigen::Matrix<double, 6+6, 6>& Apinv)
{
    using namespace Eigen;

//    int m = A.rows(), n = A.cols(), k = m < n ? m : n;
    Eigen::JacobiSVD< Eigen::Matrix<double, 6, 6+6> > svd(A,Eigen::ComputeFullU|Eigen::ComputeFullV);
    const JacobiSVD< Eigen::Matrix<double, 6, 6+6> >::SingularValuesType& singularValues = svd.singularValues();
    Eigen::Matrix<double, 12, 6> invSinValues;
    invSinValues.setZero();
    for (int idx = 0; idx < singularValues.size(); idx++)
    {
        if( idx < singularValues.size() )
        {
            invSinValues(idx,idx) = tol > 0 && singularValues(idx) > tol ? 1.0 / singularValues(idx) : 0.0;
        }
    }
    //std::cout << "singularValues: " << singularValues << std::endl;
    //std::cout << "invSinValues : " << invSinValues << std::endl;
    Eigen::Matrix<double, 12, 12> V = svd.matrixV();
    Eigen::Matrix<double, 6, 6> U = svd.matrixU();
    //std::cout << "V " << V << std::endl;
    //std::cout << "U " << U << std::endl;
    Apinv =  V * invSinValues * U.transpose(); // damped pseudoinverse
}

bool DynTree::estimateDoubleSupportContactForce(int left_foot_id, int right_foot_id)
{
    KDL::CoDyCo::rneaDynamicLoop(undirected_tree,q,dynamic_traversal,f_gi,f_ext,f,torques,base_residual_f);

    double tol = 1e-2;

    //now base_residual_f is locate at the root link, we want to divide this constribution in the two
    //wrenches at the feet
    computePositions();
    KDL::Frame H_root_leftFoot = X_dynamic_base[left_foot_id];
    KDL::Frame H_root_rightFoot = X_dynamic_base[right_foot_id];

    Eigen::Matrix<double, 6, 1> residual_f_eigen = KDL::CoDyCo::toEigen(base_residual_f);

    Eigen::Matrix<double, 6, 6+6> regressor;

    Eigen::Matrix<double, 6+6, 6> regressorPinv;

    Eigen::Matrix<double, 6+6, 1> unknown_wrenches;

    regressor.block<6,6>(0,0) = KDL::CoDyCo::WrenchTransformationMatrix(H_root_leftFoot);
    regressor.block<6,6>(0,6) = KDL::CoDyCo::WrenchTransformationMatrix(H_root_rightFoot);

    pseudoInverse(regressor,tol,regressorPinv);

    unknown_wrenches = regressorPinv*(residual_f_eigen);

    Eigen::Matrix<double, 6, 1> f_left_foot = unknown_wrenches.segment<6>(0);
    Eigen::Matrix<double, 6, 1> f_right_foot = unknown_wrenches.segment<6>(6);
    f_ext[left_foot_id] = KDL::CoDyCo::toKDLWrench(f_left_foot);
    f_ext[right_foot_id] = KDL::CoDyCo::toKDLWrench(f_right_foot);


    /*
    std::cout << "Known term " << base_residual_f << std::endl;
    std::cout << "Known term " << residual_f_eigen << std::endl;
    */
    /*
    std::cout << "Regressor " << regressor << std::endl;
    std::cout << "Regressor pinv " << regressorPinv << std::endl;
    */
    /*
    std::cout << "Estimated wrenches " << unknown_wrenches << std::endl;
        std::cout << "Estimate wrench for left foot " << f_left_foot << std::endl;
    std::cout << "Estimate wrench for right foot " << f_right_foot << std::endl;
    std::cout << "Estimate wrench for left foot " << f_ext[left_foot_id] << std::endl;
    std::cout << "Estimate wrench for right foot " << f_ext[right_foot_id] << std::endl;

    std::cout << "left foot id " << getLinkIndex("l_foot") << " used id " << left_foot_id << std::endl;
    std::cout << "right foot id " << getLinkIndex("r_foot") << " used id " << right_foot_id << std::endl;

    std::cout << "tree         : " << undirected_tree.toString() << std::endl;
    std::cout << "serialization: " <<  undirected_tree.getSerialization().toString() << std::endl;
    */
    return true;
}

bool DynTree::dynamicRNEA()
{
    int ret;
    //ret = rneaDynamicLoop(undirected_tree,q,dynamic_traversal,v,a,f_ext,f,torques,base_residual_f);
    ret = KDL::CoDyCo::rneaDynamicLoop(undirected_tree,q,dynamic_traversal,f_gi,f_ext,f,torques,base_residual_f);
    //Check base force: if estimate contact was called, it should be zero
    if( are_contact_estimated == true )
    {
        //If the force were estimated wright
        #ifndef NDEBUG
        /*
        std::cout << "q:   " << q.data << std::endl;
        std::cout << "dq:  " << dq.data << std::endl;
        std::cout << "ddq: " << ddq.data << std::endl;
        for(int i=0; i < f_ext.size(); i++ ) { std::cout << "f_ext[" << i << "]: " << f_ext[i] << std::endl; }
        */
        //std::cerr << "base_residual_f.force.Norm " << base_residual_f.force.Norm() << std::endl;
        //std::cerr << "base_residual_f.force.Norm " << base_residual_f.torque.Norm() << std::endl;

        #endif
        if(  base_residual_f.force.Norm() > 1e-5 )
        {
            std::cout << "iDynTree WARNING: base_residual_f.force.Norm() is " << base_residual_f.force.Norm() << " instead of zero." << std::endl;
            ret = -1;
        }
        if(  base_residual_f.torque.Norm() > 1e-5 )
        {
            std::cout << "iDynTree WARNING: base_residual_f.torque.Norm() is " << base_residual_f.torque.Norm() << " instead of zero." << std::endl;
            ret = -1;
        }
        //Note: this (that no residual appears happens only for the proper selection of the provided dynContactList
    }
    return ret >= 0;
}

////////////////////////////////////////////////////////////////////////
////// COM related methods
////////////////////////////////////////////////////////////////////////


KDL::Vector DynTree::getCOMKDL(int link_index) const
{
    if( (link_index < 0 || link_index >= (int)undirected_tree.getNrOfLinks()) && link_index != -1 )
    {
        std::cerr << "DynTree::getCOM: link index " << link_index <<  " out of bounds" << std::endl;
        return KDL::Vector(0.0,0.0,0.0);
    }
    if( (int)subtree_COM.size() != getNrOfLinks() ) { subtree_COM.resize(getNrOfLinks()); }
    if( (int)subtree_mass.size() != getNrOfLinks() ) { subtree_mass.resize(getNrOfLinks()); }

    KDL::Vector com_world, com_return;
    KDL::CoDyCo::GeneralizedJntPositions q_fb(world_base_frame,q);
    getCenterOfMassLoop(undirected_tree,q_fb,dynamic_traversal,subtree_COM,subtree_mass,com_world);


    if( link_index == -1 ) {
        com_return = com_world;
    } else {
        // com_return =  H_link_base*H_base_world*com_world
        computePositions();
        com_return = X_dynamic_base[link_index].Inverse(world_base_frame.Inverse(com_world));
    }

    /*
     std::vector<KDL::Frame> Xb;

     Xb.resize(undirected_tree.getNrOfLinks());

    KDL::RigidBodyInertia total_inertia;
    getMomentumJacobianLoop(undirected_tree,q,dynamic_traversal,Xb,momentum_jac_buffer,com_jac_buffer,momentum_jacobian,total_inertia,part_id);
    */

    return com_return;
}

yarp::sig::Vector DynTree::getCOM(int link_index) const
{
    KDL::Vector com_return = getCOMKDL(link_index);
    size_t com_return_size = sizeof(com_return)/sizeof(double);
    com_yarp.resize(com_return_size,0);

    memcpy(com_yarp.data(),com_return.data,3*sizeof(double));

    return com_yarp;
}


bool DynTree::getCOMJacobianKDL(KDL::Jacobian & jac)
{
    KDL::CoDyCo::MomentumJacobian dummy;
    if( (int)com_jacobian.columns() != 6+getNrOfDOFs() ) { com_jacobian.resize(6+getNrOfDOFs()); }
    if( (int)momentum_jacobian.columns() != 6+getNrOfDOFs() ) { momentum_jacobian.resize(6+getNrOfDOFs()); }
    SetToZero(com_jacobian);
    SetToZero(momentum_jacobian);
    bool result= getCOMJacobianKDL(com_jacobian,momentum_jacobian);
    jac=com_jacobian;

    return result;

}

bool DynTree::getCOMJacobianKDL(KDL::Jacobian & com_jac,  KDL::CoDyCo::MomentumJacobian & momentum_jac)
{
    if( (int)com_jac.columns() != 6+getNrOfDOFs() ) { com_jac.resize(6+getNrOfDOFs()); }
    if( (int)momentum_jac.columns() != 6+getNrOfDOFs() ) { momentum_jac.resize(6+getNrOfDOFs()); }
    if( (int)com_jac_buffer.columns() != 6+getNrOfDOFs() ) { com_jac_buffer.resize(6+getNrOfDOFs()); }
    if( (int)momentum_jac_buffer.columns() != 6+getNrOfDOFs() ) { momentum_jac_buffer.resize(6+getNrOfDOFs()); }

    SetToZero(com_jac);
    SetToZero(momentum_jac);
    SetToZero(com_jac_buffer);
    SetToZero(momentum_jac_buffer);

    int part_id;
    part_id = -1;


    computePositions();

    KDL::RigidBodyInertia base_total_inertia;

    getMomentumJacobianLoop(undirected_tree,q,dynamic_traversal,X_dynamic_base,momentum_jac,com_jac_buffer,momentum_jac_buffer,base_total_inertia);

    /*
    std::cout << "Total Inertia for part " << part_name << " : " << std::endl
              << " mass : " << base_total_inertia.getMass() << " " << std::endl
              << " cog "  << base_total_inertia.getCOG() <<  std::endl
              <<  " inertia around the origin " << Eigen::Map<Eigen::Matrix3d>(base_total_inertia.getRotationalInertia().data) << std::endl;
    */

    momentum_jac.changeRefFrame(KDL::Frame(world_base_frame.M));

    total_inertia = KDL::Frame(world_base_frame.M)*base_total_inertia;

    if( total_inertia.getMass() == 0 )
    {
        std::cerr << "iDynTree::getCOMJacobian error: Tree has no mass " << std::endl;
        return false;
    }

    momentum_jac.changeRefPoint(total_inertia.getCOG());

    /** \todo add a meaniful transformation for the rotational part of the jacobian */
    //KDL::CoDyCo::divideJacobianInertia(momentum_jacobian,total_inertia,com_jacobian);
    com_jac.data = momentum_jacobian.data/total_inertia.getMass();

    //As in iDynTree the base twist is expressed in the world frame, the first six columns are always the identity
    com_jac.setColumn(0,KDL::Twist(KDL::Vector(1,0,0),KDL::Vector(0,0,0)).RefPoint(total_inertia.getCOG()));
    com_jac.setColumn(1,KDL::Twist(KDL::Vector(0,1,0),KDL::Vector(0,0,0)).RefPoint(total_inertia.getCOG()));
    com_jac.setColumn(2,KDL::Twist(KDL::Vector(0,0,1),KDL::Vector(0,0,0)).RefPoint(total_inertia.getCOG()));
    com_jac.setColumn(3,KDL::Twist(KDL::Vector(0,0,0),KDL::Vector(1,0,0)).RefPoint(total_inertia.getCOG()));
    com_jac.setColumn(4,KDL::Twist(KDL::Vector(0,0,0),KDL::Vector(0,1,0)).RefPoint(total_inertia.getCOG()));
    com_jac.setColumn(5,KDL::Twist(KDL::Vector(0,0,0),KDL::Vector(0,0,1)).RefPoint(total_inertia.getCOG()));

    momentum_jac.changeRefPoint(-total_inertia.getCOG());

    return true;
}


bool DynTree::getCOMJacobian(yarp::sig::Matrix & jac)
{
    return getCOMJacobian(jac,com_jacobian_buffer);
}

bool DynTree::getCOMJacobian(yarp::sig::Matrix & jac, yarp::sig::Matrix & momentum_jac)
{
    if( (int)com_jacobian.columns() != 6+getNrOfDOFs() ) { com_jacobian.resize(6+getNrOfDOFs()); }
    if( (int)momentum_jacobian.columns() != 6+getNrOfDOFs() ) { momentum_jacobian.resize(6+getNrOfDOFs()); }
    if( (int)com_jac_buffer.columns() != 6+getNrOfDOFs() ) { com_jac_buffer.resize(6+getNrOfDOFs()); }
    if( (int)momentum_jac_buffer.columns() != 6+getNrOfDOFs() ) { momentum_jac_buffer.resize(6+getNrOfDOFs()); }

    if( jac.rows() != (int)(6) || jac.cols() != (int)(6+undirected_tree.getNrOfDOFs()) ) {
        jac.resize(6,6+undirected_tree.getNrOfDOFs());
    }

    if( momentum_jac.rows() != (int)(6) || momentum_jac.cols() != (int)(6+undirected_tree.getNrOfDOFs()) ) {
        momentum_jac.resize(6,6+undirected_tree.getNrOfDOFs());
    }

    SetToZero(com_jacobian);
    SetToZero(momentum_jacobian);
    SetToZero(com_jac_buffer);
    SetToZero(momentum_jac_buffer);
    jac.zero();
    momentum_jac.zero();

    getCOMJacobianKDL(com_jacobian,momentum_jacobian);


    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_jacobian(jac.data(),jac.rows(),jac.cols());

    mapped_jacobian = com_jacobian.data;

    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_momentum_jacobian(momentum_jac.data(),momentum_jac.rows(),momentum_jac.cols());

    mapped_momentum_jacobian = momentum_jacobian.data;

    return true;
}



bool DynTree::getCentroidalMomentumJacobian(yarp::sig::Matrix & momentum_jac)
{
    if( (int)momentum_jacobian.columns() != 6+getNrOfDOFs() ) { momentum_jacobian.resize(6+getNrOfDOFs()); }
    if( (int)momentum_jac_buffer.columns() != 6+getNrOfDOFs() ) { momentum_jac_buffer.resize(6+getNrOfDOFs()); }

    if( momentum_jac.rows() != (int)(6) || momentum_jac.cols() != (int)(6+undirected_tree.getNrOfDOFs()) ) {
        momentum_jac.resize(6,6+undirected_tree.getNrOfDOFs());
    }

    momentum_jac.zero();
    SetToZero(momentum_jacobian);
    SetToZero(momentum_jac_buffer);

    computePositions();

    KDL::RigidBodyInertia base_total_inertia;

    getMomentumJacobianLoop(undirected_tree,q,dynamic_traversal,X_dynamic_base,momentum_jacobian,com_jac_buffer,momentum_jac_buffer,base_total_inertia);


    //Fixed base mass matrix (the n x n bottom right submatrix) is ok in this way
    //but the other submatrices must be changed, as iDynTree express all velocities/accelerations (also the base one) in world orientation
    //while kdl_codyco express the velocities in base orientation
    KDL::Frame world_base_rotation = KDL::Frame(world_base_frame.M);

    //As the transformation is a rotation, the adjoint trasformation is the same for both twist and wrenches
    //Additionally, the inverse of the adjoint matrix is simply the transpose
    Eigen::Matrix< double, 6, 6> world_base_rotation_adjoint_transformation = KDL::CoDyCo::WrenchTransformationMatrix(world_base_rotation);


    //Modification of 6x6 left upper submatrix (spatial inertia)
    // doing some moltiplication by zero (inefficient? )
    //fb_jnt_mass_matrix.data.block<6,6>(0,0) = world_base_rotation_adjoint_transformation*fb_jnt_mass_matrix.data.block<6,6>(0,0);
    //fb_jnt_mass_matrix.data.block<6,6>(0,0) = fb_jnt_mass_matrix.data.block<6,6>(0,0)*world_base_rotation_adjoint_transformation.transpose();
    Eigen::Matrix<double,6,6> buffer_mat_six_six =  world_base_rotation_adjoint_transformation* momentum_jacobian.data.block<6,6>(0,0);
    momentum_jacobian.data.block<6,6>(0,0) = buffer_mat_six_six*(world_base_rotation_adjoint_transformation.transpose());

    for(int dof=0; dof < undirected_tree.getNrOfDOFs(); dof++ ) {
        //fb_jnt_mass_matrix.data.block<6,1>(0,6+dof) = world_base_rotation_adjoint_transformation*fb_jnt_mass_matrix.data.block<6,1>(0,6+dof);
        //fb_jnt_mass_matrix.data.block<1,6>(6+dof,0) = fb_jnt_mass_matrix.data.block<6,1>(0,6+dof).transpose();
        Eigen::Matrix<double,6,1> buffer_vec_six = world_base_rotation_adjoint_transformation*momentum_jacobian.data.block<6,1>(0,6+dof);
        momentum_jacobian.data.block<6,1>(0,6+dof) = buffer_vec_six.transpose();
    }


    total_inertia = (KDL::Frame(world_base_frame.M))*base_total_inertia;

    momentum_jacobian.changeRefPoint(total_inertia.getCOG());

    //std::cout << "Total inertia test " << total_inertia.RefPoint(total_inertia.getCOG()).getCOG() << std::endl;

    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_momentum_jacobian(momentum_jac.data(),momentum_jac.rows(),momentum_jac.cols());

    mapped_momentum_jacobian = momentum_jacobian.data;

    return true;
}


yarp::sig::Vector DynTree::getCentroidalMomentum()
{
    yarp::sig::Vector momentum_yarp(6);
    yarp::sig::Vector lin_vel(3), ang_vel(3);
    yarp::sig::Vector com(3);
    /** \todo add controls like for computePositions() */
    kinematicRNEA();
    computePositions();

    double m = 0;
    KDL::Wrench mom;
    KDL::Wrench mom_world;
    for(int i=0; i < getNrOfLinks(); i++ ) {
        double m_i = undirected_tree.getLink(i)->getInertia().getMass();
        mom += (X_dynamic_base[i]*(undirected_tree.getLink(i)->getInertia()*v[i]));
        m += m_i;
        //std::cout << v[i] << std::endl;
        //std::cout << mom << std::endl;
    }

    mom_world = world_base_frame*mom;

    com = getCOM();
    KDL::Vector com_kdl;

    YarptoKDL(com,com_kdl);


    KDL::Wrench mom_out =  mom_world.RefPoint(com_kdl);

    KDLtoYarp(mom_out.force,lin_vel);
    KDLtoYarp(mom_out.torque,ang_vel);
    momentum_yarp.setSubvector(0,lin_vel);
    momentum_yarp.setSubvector(3,ang_vel);
    return momentum_yarp;
}


yarp::sig::Vector DynTree::getVelCOM()
{
    if( com_yarp.size() != 3 ) { com_yarp.resize(3); }

    /** \todo add controls like for computePositions() */
    kinematicRNEA();
    computePositions();

    double m = 0;
    KDL::Vector mdcom;
    KDL::Vector mdcom_world;
    KDL::Vector dcom_world;
    for(int i=0; i < getNrOfLinks(); i++ ) {
        double m_i = undirected_tree.getLink(i)->getInertia().getMass();
        KDL::Vector com_i = undirected_tree.getLink(i)->getInertia().getCOG();
        mdcom += X_dynamic_base[i].M*(m_i*(v[i].RefPoint(com_i)).vel);
        m += m_i;
    }

    mdcom_world = world_base_frame.M*mdcom;

    dcom_world = mdcom_world/m;

    memcpy(com_yarp.data(),dcom_world.data,3*sizeof(double));

    return com_yarp;
}

yarp::sig::Vector DynTree::getAccCOM()
{
    if( com_yarp.size() != 3 ) { com_yarp.resize(3); }
    getAccCOM(com_yarp);
    return com_yarp;
}


bool DynTree::getAccCOM(yarp::sig::Vector & com_acceleration)
{
  if( com_acceleration.size() != 3 ) { com_acceleration.resize(3); }

    /** \todo add controls like for computePositions() */
    kinematicRNEA();
    computePositions();

    double m = 0; /// \< Mass of the complete robot
    KDL::Vector m_d2com; /// \< sum (expressed in base frame) of all the com accelerations of a link multiplied by the link mass
    KDL::Vector m_d2com_world; /// \< as m_d2com, but expressed with respect to world orientation
    KDL::Vector d2com_world; /// \< com acceleration expressed in the world frame
    for(int i=0; i < getNrOfLinks(); i++ ) {
        double m_i = undirected_tree.getLink(i)->getInertia().getMass();
        KDL::Vector com_i = undirected_tree.getLink(i)->getInertia().getCOG();
        KDL::Twist vel_link_com = v[i].RefPoint(com_i);
        KDL::Twist spatial_acc_link_com = a[i].RefPoint(com_i);
        KDL::Twist classical_acc_link_com;
        KDL::CoDyCo::spatialToConventionalAcceleration(spatial_acc_link_com,vel_link_com,classical_acc_link_com);
        m_d2com += X_dynamic_base[i].M*(m_i*classical_acc_link_com.vel);
        m += m_i;
    }

    m_d2com_world = world_base_frame.M*m_d2com;

    d2com_world = m_d2com_world/m;

    memcpy(com_acceleration.data(),d2com_world.data,3*sizeof(double));


    return true;
}

yarp::sig::Vector DynTree::getMomentum()
{
    yarp::sig::Vector momentum_yarp(6);
    yarp::sig::Vector lin_vel(3), ang_vel(3);
    /** \todo add controls like for computePositions() */
    kinematicRNEA();
    computePositions();

    double m = 0;
    KDL::Wrench mom;
    KDL::Wrench mom_world;
    for(int i=0; i < getNrOfLinks(); i++ ) {
        double m_i = undirected_tree.getLink(i)->getInertia().getMass();
        mom += (X_dynamic_base[i]*(undirected_tree.getLink(i)->getInertia()*v[i]));
        m += m_i;
    }

    mom_world = world_base_frame*mom;

    KDLtoYarp(mom_world.force,lin_vel);
    KDLtoYarp(mom_world.torque,ang_vel);
    momentum_yarp.setSubvector(0,lin_vel);
    momentum_yarp.setSubvector(3,ang_vel);
    return momentum_yarp;
}


////////////////////////////////////////////////////////////////////////
////// Jacobian related methods
////////////////////////////////////////////////////////////////////////
bool DynTree::getJacobianKDL(const int link_index, KDL::Jacobian & abs_jac, bool local)
{
    if( link_index < 0 ||
        link_index >= (int)undirected_tree.getNrOfLinks() )
    {
        std::cerr << "DynTree::getJacobian: link index " << link_index <<  " out of bounds" << std::endl;
        return false;
    }

    if( abs_jacobian.rows() != 6 ||
        abs_jacobian.columns() != 6+undirected_tree.getNrOfDOFs() )
    {
        abs_jacobian.resize(6+undirected_tree.getNrOfDOFs());
    }

    getFloatingBaseJacobianLoop(undirected_tree,
                                KDL::CoDyCo::GeneralizedJntPositions(world_base_frame,q),
                                dynamic_traversal,
                                link_index,
                                abs_jacobian);

    return true;
}

bool DynTree::getJacobian(const int link_index, yarp::sig::Matrix & jac, bool local)
{
    if( link_index < 0 ||
        link_index >= (int)undirected_tree.getNrOfLinks() )
    {
        std::cerr << "DynTree::getJacobian: link index " << link_index
                  <<  " out of bounds" << std::endl;
        return false;

    }
    if( jac.rows() != (int)(6) || jac.cols() != (int)(6+undirected_tree.getNrOfDOFs()) ) {
        jac.resize(6,6+undirected_tree.getNrOfDOFs());
    }

    getJacobianKDL(link_index,abs_jacobian,local);

    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_jacobian(jac.data(),jac.rows(),jac.cols());

    mapped_jacobian = abs_jacobian.data;

    return true;
}

bool DynTree::getRelativeJacobianKDL(const int jacobian_distal_link,
                                     const int jacobian_base_link,
                                     KDL::Jacobian & rel_jac,
                                     bool global)
{
    if( rel_jac.rows() != 6 ||
        rel_jac.columns() != undirected_tree.getNrOfDOFs() )
    {
        rel_jac.resize(undirected_tree.getNrOfDOFs());
    }

    if( jacobian_base_link < 0 ||
        jacobian_base_link >= this->getNrOfLinks() )
    {
        std::cerr << "[ERROR] DynTree::getRelativeJacobianKDL : jacobian_base_link "
                  << jacobian_base_link << " is out of bounds" << std::endl;
        return false;
    }

    if( jacobian_distal_link < 0 ||
        jacobian_distal_link >= this->getNrOfLinks() )
    {
        std::cerr << "[ERROR] DynTree::getRelativeJacobianKDL : jacobian_distal_link "
                  << jacobian_distal_link << " is out of bounds" << std::endl;
        return false;
    }

    /*
     if the specified jacobian_base_link is the base in dynamic_traversal, kinematic_traversal or rel_jacobian_traversal
     use the traversal already available, otherwise overwrite the rel_jacobian_traversal with the traversal with base at jacobian_base_link
    */
    KDL::CoDyCo::Traversal * p_traversal;

    if( dynamic_traversal.getBaseLink()->getLinkIndex() == jacobian_base_link ) {
        p_traversal = &dynamic_traversal;
    } else if ( kinematic_traversal.getBaseLink()->getLinkIndex() == jacobian_base_link ) {
        p_traversal = &kinematic_traversal;
    } else {
       if( rel_jacobian_traversal.getNrOfVisitedLinks() != (int)undirected_tree.getNrOfLinks() ||
           rel_jacobian_traversal.getBaseLink()->getLinkIndex() != jacobian_base_link  ) {
            int ret_ct = undirected_tree.compute_traversal(rel_jacobian_traversal,jacobian_base_link);
            if( ret_ct != 0 )
            {
                std::cerr << "[ERROR] DynTree::getRelativeJacobianKDL : compute_traversal failed for jacobian_base_link equal to"
                          << jacobian_base_link << std::endl;
                return false;
            }
       }
       p_traversal = &rel_jacobian_traversal;
    }

    assert( p_traversal->getBaseLink()->getLinkIndex() == jacobian_base_link );

    getRelativeJacobianLoop(undirected_tree,q,*p_traversal,jacobian_distal_link,rel_jacobian);

    if( global ) {
        computePositions();
        rel_jac.changeRefFrame(world_base_frame*X_dynamic_base[jacobian_distal_link]);
    }

    return true;
}

bool DynTree::getRelativeJacobian(const int jacobian_distal_link,
                                  const int jacobian_base_link,
                                  yarp::sig::Matrix & jac,
                                  bool global)
{
    if( jac.rows() != (int)(6) ||
        jac.cols() != (int)(undirected_tree.getNrOfDOFs()) )
    {
        jac.resize(6,undirected_tree.getNrOfDOFs());
    }


    if( ! getRelativeJacobianKDL(jacobian_distal_link,jacobian_base_link,rel_jacobian,global) )
    {
        return false;
    }

    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_jacobian(jac.data(),jac.rows(),jac.cols());

    mapped_jacobian = rel_jacobian.data;

    return true;
}

////////////////////////////////////////////////////////////////////////
////// Mass Matrix (CRBA) related methods
////////////////////////////////////////////////////////////////////////
bool DynTree::getFloatingBaseMassMatrix(yarp::sig::Matrix & fb_mass_matrix)
{
    //If the incoming matrix have the wrong number of rows/colums, resize it
    if( fb_mass_matrix.rows() != (int)(6+undirected_tree.getNrOfDOFs())
        || fb_mass_matrix.cols() != (int)(6+undirected_tree.getNrOfDOFs()) )
    {
        fb_mass_matrix.resize(6+undirected_tree.getNrOfDOFs(),6+undirected_tree.getNrOfDOFs());
    }

    fb_mass_matrix.zero();

    //Calculate the result directly in the output matrix
    /**
     * \todo TODO modify crba loops in a way that it can run directly in the fb_mass_matrix.data();
     */
    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_mass_matrix(fb_mass_matrix.data(),fb_mass_matrix.rows(),fb_mass_matrix.cols());

    if( fb_jnt_mass_matrix.rows() != (6+undirected_tree.getNrOfDOFs())
        || fb_jnt_mass_matrix.columns() != (6+undirected_tree.getNrOfDOFs()) )
    {
        fb_jnt_mass_matrix.resize(6+undirected_tree.getNrOfDOFs());
    }


    if( subtree_crbi.size() != undirected_tree.getNrOfLinks() ) {
        subtree_crbi.resize(undirected_tree.getNrOfLinks());

    };

    KDL::CoDyCo::GeneralizedJntPositions q_fb(world_base_frame,q);
    int successfull_return = KDL::CoDyCo::crba_floating_base_loop(undirected_tree,
                                                                  dynamic_traversal,
                                                                  q_fb,
                                                                  subtree_crbi,
                                                                  fb_jnt_mass_matrix);

    if( successfull_return != 0 ) {
        return false;
    }


    //This copy does not exploit the matrix sparsness..
    //but I guess that exploiting it would lead to slower code
    assert(fb_jnt_mass_matrix.rows() == fb_jnt_mass_matrix.columns());
    assert(fb_mass_matrix.rows() == (int)fb_jnt_mass_matrix.rows());
    assert(fb_mass_matrix.cols() == (int)fb_jnt_mass_matrix.columns());
    mapped_mass_matrix = fb_jnt_mass_matrix.data;

    return true;
}



////////////////////////////////////////////////////////////////////////
////// Regressor related methods
////////////////////////////////////////////////////////////////////////
bool DynTree::getDynamicsRegressor(yarp::sig::Matrix & mat)
{
    //If the incoming matrix have the wrong number of rows/colums, resize it
    if( mat.rows() != (int)(6+undirected_tree.getNrOfDOFs()) || mat.cols() != (int)(10*undirected_tree.getNrOfLinks()) ) {
        mat.resize(6+undirected_tree.getNrOfDOFs(),10*undirected_tree.getNrOfLinks());
    }

    //Calculate the result directly in the output matrix
    /**
     * \todo check that X_b,v and are computed
     */
    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_dynamics_regressor(mat.data(),mat.rows(),mat.cols());

    Eigen::MatrixXd dynamics_regressor;
    dynamics_regressor.resize(6+undirected_tree.getNrOfDOFs(),10*undirected_tree.getNrOfLinks());

    computeXWorld();
    dynamicsRegressorLoop(undirected_tree,q,dynamic_traversal,X_world,v,a,dynamics_regressor);

    mapped_dynamics_regressor = dynamics_regressor;

    return true;
}

bool DynTree::getDynamicsParameters(yarp::sig::Vector & vec)
{
    if( vec.size() != 10*undirected_tree.getNrOfLinks() ) {
        vec.resize(10*undirected_tree.getNrOfLinks());
    }

    Eigen::Map< Eigen::VectorXd > mapped_vector(vec.data(),10*undirected_tree.getNrOfLinks());
    Eigen::VectorXd inertial_parameters;
    inertial_parameters.resize(10*undirected_tree.getNrOfLinks());

    inertialParametersVectorLoop(undirected_tree,inertial_parameters);

    mapped_vector = inertial_parameters;

    return true;
}

int DynTree::getNrOfDOFs() const
{
    return undirected_tree.getNrOfDOFs();
}

int DynTree::getNrOfLinks() const
{
    return undirected_tree.getNrOfLinks();
}

//\todo FIXME TODO properly implement frame support
int DynTree::getNrOfFrames() const
{
    return getNrOfLinks();
}

int DynTree::getNrOfFTSensors() const
{
    return NrOfFTSensors;
}

int DynTree::getNrOfIMUs() const
{
    return 1;
}

int DynTree::getLinkIndex(const std::string & link_name) const
{
    KDL::CoDyCo::LinkMap::const_iterator link_it = undirected_tree.getLink(link_name);
    if( link_it == undirected_tree.getInvalidLinkIterator() ) { std::cerr << "DynTree::getLinkIndex : link " << link_name << " not found" << std::endl; return -1; }
    return link_it->getLinkIndex();
}


bool DynTree::getLinkName(const int link_index, std::string & link_name) const
{
    if( link_index < 0 || link_index >= this->getNrOfLinks() )
    {
        return false;
    }
    link_name = undirected_tree.getLink(link_index)->getName();
    return true;
}

int DynTree::getFrameIndex(const std::string & frame_name) const
{
    return getLinkIndex(frame_name);
}

bool DynTree::getFrameName(const int frame_index, std::string & frame_name) const
{
    return getLinkName(frame_index, frame_name);
}


int DynTree::getDOFIndex(const std::string & dof_name) const
{
    KDL::CoDyCo::JunctionMap::const_iterator junction_it = undirected_tree.getJunction(dof_name);
    if( junction_it == undirected_tree.getInvalidJunctionIterator() || junction_it->getNrOfDOFs() != 1 ) { std::cerr << "DynTree::getDOFIndex : DOF " << dof_name << " not found" << std::endl; return -1; }
    return junction_it->getDOFIndex();
}

bool DynTree::getDOFName(const int dof_index, std::string & dof_name) const
{
    KDL::CoDyCo::JunctionMap::const_iterator junction_it = undirected_tree.getJunction(dof_index);
    if( junction_it == undirected_tree.getInvalidJunctionIterator()
        || junction_it->getNrOfDOFs() != 1 )
    {
        std::cerr << "DynTree::getDOFName : DOF " << dof_name << " not found" << std::endl;
        return false;
    }
    dof_name = junction_it->getName();
    return true;
}

int DynTree::getJunctionIndex(const std::string & junction_name) const
{
    KDL::CoDyCo::JunctionMap::const_iterator junction_it = undirected_tree.getJunction(junction_name);
    if( junction_it == undirected_tree.getInvalidJunctionIterator() ) { std::cerr << "DynTree::getJunctionIndex : Junction " << junction_name << " not found" << std::endl; return -1; }
    return junction_it->getJunctionIndex();
}

bool DynTree::getJunctionName(const int junction_index, std::string & junction_name) const
{
    KDL::CoDyCo::JunctionMap::const_iterator junction_it = undirected_tree.getJunction(junction_index);
    if( junction_it == undirected_tree.getInvalidJunctionIterator() )
    {
        std::cerr << "DynTree::getJunctionName : Junction " << junction_name << " not found" << std::endl;
        return false;
    }
    junction_name = junction_it->getName();
    return true;
}

// \todo TODO FIXME implement this method
bool DynTree::getFTSensorName(const int /*junction_index*/, std::string & /*junction_name*/) const
{
    return false;
}

int DynTree::getFTSensorIndex(const std::string & ft_name) const
{
    for(unsigned int ft_index = 0; ft_index < NrOfFTSensors; ft_index++ )
    {
        if(ft_name == m_joint_sensor_names[ft_index] )
        {
            return ft_index;
        }
    }

    return -1;

    //return ft_list.getFTSensorID(junction_it->getJunctionIndex());
}

int DynTree::getIMUIndex(const std::string & imu_name) const
{
    if( imu_name == kinematic_traversal.getBaseLink()->getName() ) {
        return 0;
    } else {
         std::cerr << "DynTree::getIMUIndex : IMU " << imu_name << " not found" << std::endl;
         return -1;
    }
}

// \todo TODO FIXME implement this method
bool DynTree::getIMUName(const int /*junction_index*/, std::string & /*junction_name*/) const
{
    return false;
}

bool DynTree::loadJointLimitsFromURDFFile(std::string urdfFile, KDL::CoDyCo::UndirectedTree undirectedTree, yarp::sig::Vector &yarpJointMinLimit, yarp::sig::Vector &yarpJointMaxLimit) {

        unsigned int NrOfDOFs = undirectedTree.getNrOfDOFs();
        KDL::JntArray kdlJointMinLimit(NrOfDOFs), kdlJointMaxLimit(NrOfDOFs);
        std::vector<std::string> jointLimitsNames;
        ::iDynTree::jointPosLimitsFromUrdfFile(urdfFile, jointLimitsNames, kdlJointMinLimit, kdlJointMaxLimit);

        if (yarpJointMinLimit.size() != NrOfDOFs)
            yarpJointMinLimit.resize(NrOfDOFs);
        if (yarpJointMaxLimit.size() != NrOfDOFs)
            yarpJointMaxLimit.resize(NrOfDOFs);

        KDL::CoDyCo::TreeSerialization treeSerialization = undirectedTree.getSerialization();

        for (int dof = 0; dof < NrOfDOFs; dof++)
        {
            std::string dof_name = treeSerialization.getDOFName(dof);
            for (int lim = 0; lim < jointLimitsNames.size(); lim++)
            {
                if (jointLimitsNames[lim] == dof_name)
                {
                    yarpJointMinLimit[dof] = kdlJointMinLimit(lim);
                    yarpJointMaxLimit[dof] = kdlJointMaxLimit(lim);
                    break;
                }
            }
        }

        return true;
    }

}
}
