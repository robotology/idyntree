/**
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#include <cmath>
#include <cfloat>

#include "dynamicRegressorGenerator.hpp"
#include "dirl_utils.hpp"

#include "kdl_codyco/regressor_utils.hpp"
#include "kdl_codyco/regressor_loops.hpp"
#include "kdl_codyco/rnea_loops.hpp"
#include "kdl_codyco/position_loops.hpp"
#include "iDynTree/Sensors/Sensors.hpp"
#include "kdl_codyco/KDLConversions.h"
#include "iDynTree/Core/Wrench.h"
#include <iostream>

#include <algorithm>
#include <deque>

#include <Eigen/LU>

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

namespace KDL {
namespace CoDyCo {
namespace Regressors {

double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

// DynamicsRegressorParametersList helpers

DynamicRegressorGenerator::DynamicRegressorGenerator(const KDL::CoDyCo::UndirectedTree & _undirected_tree,
                                                     const iDynTree::SensorsList & _sensors_tree,
                                                     std::string kinematic_base,
                                                     bool ft_sensor_offset,
                                                      std::vector< std::string > _fake_links_names,
                                                      const bool _verbose
                                                    ):
                                                      undirected_tree(_undirected_tree),
                                                      sensorsList(_sensors_tree),
                                                      regressors_ptrs(0),
                                                      consider_ft_offset(ft_sensor_offset),
                                                      fake_links_names(_fake_links_names),
                                                      verbose(_verbose)
{
    NrOfFakeLinks = fake_links_names.size();
    NrOfDOFs = undirected_tree.getNrOfDOFs();
    NrOfRealLinks_gen = undirected_tree.getNrOfLinks()-NrOfFakeLinks;



    //Initially no regressor is installed, so the number of outputs is zero
    NrOfOutputs = 0;


    assert((int)undirected_tree.getNrOfDOFs() == NrOfDOFs);
    assert((int)undirected_tree.getNrOfLinks() == NrOfFakeLinks+NrOfRealLinks_gen);

    q = KDL::JntArray(NrOfDOFs);
    dq = KDL::JntArray(NrOfDOFs);
    ddq = KDL::JntArray(NrOfDOFs);

    measured_torques = KDL::JntArray(NrOfDOFs);

    kinematic_traversal = KDL::CoDyCo::Traversal();
    dynamic_traversal = KDL::CoDyCo::Traversal();

    sensorMeasures.setNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE,sensorsList.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE));

    X_dynamic_base = std::vector<KDL::Frame>(undirected_tree.getNrOfLinks());
    v = std::vector<KDL::Twist>(undirected_tree.getNrOfLinks());
    a = std::vector<KDL::Twist>(undirected_tree.getNrOfLinks());

    //Computing the traversal for kinematic information
    int ret;
    if( kinematic_base.length() == 0 ) {
        //default case, using the base of the tree as the kinematic base
        ret = undirected_tree.compute_traversal(kinematic_traversal);
    } else {
        ret = undirected_tree.compute_traversal(kinematic_traversal,kinematic_base);
    }
    assert( ret >= 0);
    if( ret < 0 ) { return; }

    //Computing the default (dynamic) traversal
    ret = undirected_tree.compute_traversal(dynamic_traversal);
    assert( ret >= 0 );
    if( ret < 0 ) { return; }


    //Take into account the real or fake links
    is_link_real.resize(undirected_tree.getNrOfLinks(),true);
    linkIndeces2regrColumns.resize(undirected_tree.getNrOfLinks(),-1);
    regrColumns2linkIndeces.resize(NrOfRealLinks_gen,-1);
    for(int ll=0; ll < (int)fake_links_names.size(); ll++ ) {
        KDL::CoDyCo::LinkMap::const_iterator link_it = undirected_tree.getLink(fake_links_names[ll]);
        if( link_it == undirected_tree.getInvalidLinkIterator() )
        {
            NrOfDOFs = NrOfRealLinks_gen = NrOfOutputs = 0; return;

        }
        is_link_real[link_it->getLinkIndex()]=false;
    }

    int regressor_link_index = 0;
    for(int link_index=0; link_index < (int)undirected_tree.getNrOfLinks(); link_index++ ) {
        if( is_link_real[link_index] ) {
            assert( regressor_link_index < NrOfRealLinks_gen );
            linkIndeces2regrColumns[link_index] = regressor_link_index;
            regrColumns2linkIndeces[regressor_link_index] = link_index;
            regressor_link_index++;
        } else {
           linkIndeces2regrColumns[link_index] = -1;
        }
    }
    assert(regressor_link_index == NrOfRealLinks_gen);
}

int DynamicRegressorGenerator::changeDynamicBase(std::string new_dynamic_base_name)
{
    KDL::CoDyCo::LinkMap::const_iterator link_it = undirected_tree.getLink(new_dynamic_base_name);
    if( link_it == undirected_tree.getInvalidLinkIterator() ) {
        if( verbose ) { std::cerr << "DynamicRegressorGenerator::changeDynamicBase error " << new_dynamic_base_name << " link not found" << std::endl; }
        return -1;
    }
    int ret = undirected_tree.compute_traversal(dynamic_traversal,link_it->getLinkIndex());
    if( ret != 0 ) {
        if( verbose ) { std::cerr << "DynamicRegressorGenerator::changeDynamicBase error " << new_dynamic_base_name << " compute traversal failed" << std::endl; }
        return -2;
    }
    return 0;
}

int DynamicRegressorGenerator::changeKinematicBase(std::string new_kinematic_base_name)
{
    KDL::CoDyCo::LinkMap::const_iterator link_it = undirected_tree.getLink(new_kinematic_base_name);
    if( link_it == undirected_tree.getInvalidLinkIterator() )
    {
        if( verbose ) { std::cerr << "DynamicRegressorGenerator::changeKinematicBase error " << new_kinematic_base_name << " link not found" << std::endl; }
        return 1;
    }
    int ret = undirected_tree.compute_traversal(kinematic_traversal,link_it->getLinkIndex());
    if( ret != 0 )
    {
        if( verbose ) { std::cerr << "DynamicRegressorGenerator::changeKinematicBase error " << new_kinematic_base_name << " compute traversal failed" << std::endl; }
        return -2;
    }
    return 0;
}

int DynamicRegressorGenerator::getDynamicBaseIndex()
{
    return dynamic_traversal.getBaseLink()->getLinkIndex();
}

int DynamicRegressorGenerator::getKinematicBaseIndex()
{
    return kinematic_traversal.getBaseLink()->getLinkIndex();
}


int DynamicRegressorGenerator::getNrOfParameters() const
{
    return parameters_desc.getNrOfParameters();
}

int DynamicRegressorGenerator::getNrOfOutputs() const
{
    return NrOfOutputs;
}

int DynamicRegressorGenerator::getNrOfDOFs() const
{
    return undirected_tree.getNrOfDOFs();
}


int DynamicRegressorGenerator::getNrOfWrenchSensors() const
{
    return sensorsList.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);
}


std::string DynamicRegressorGenerator::getDescriptionOfParameter(int parameter_index, bool with_value, double value)
{
#ifndef NDEBUG
    //std::cout << "GetdescriptionOfParameter with argument " << parameter_index << std::endl;
#endif

    std::string elemName;

    if( parameters_desc.parameters[parameter_index].category ==
        iDynTree::Regressors::LINK_PARAM )
    {
        elemName = undirected_tree.getLink(parameters_desc.parameters[parameter_index].elemIndex)->getName();
    }

    if( parameters_desc.parameters[parameter_index].category ==
        iDynTree::Regressors::SENSOR_FT_PARAM )
    {
        elemName = sensorsList.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,parameters_desc.parameters[parameter_index].elemIndex)->getName();
    }

    std::stringstream ss;

    ss << parameters_desc.getDescriptionOfParameter(parameter_index,elemName);

    if( with_value ) {
        ss << "\t\t" << value << std::endl;
    }

    return ss.str();
}

std::string DynamicRegressorGenerator::getDescriptionOfParameters()
{
    std::stringstream ss;

    for(int parameter_index=0; parameter_index<parameters_desc.getNrOfParameters(); parameter_index++) {
        ss << getDescriptionOfParameter(parameter_index) << std::endl;
    }

    return ss.str();
}

std::string DynamicRegressorGenerator::getDescriptionOfParameters(const Eigen::VectorXd & values)
{
    std::stringstream ss;

    bool with_value = true;
    for(int parameter_index=0; parameter_index<parameters_desc.getNrOfParameters(); parameter_index++) {
        ss << getDescriptionOfParameter(parameter_index,with_value,values[parameter_index]) << std::endl;
    }

    return ss.str();
}

std::string DynamicRegressorGenerator::getDescriptionOfOutput(int output_index)
{
    std::stringstream ss;

    ss << "DynamicRegressorGenerator::getDescriptionOfOutput(" << output_index << ") : Not implemented.";

    return ss.str();
}

std::string DynamicRegressorGenerator::getDescriptionOfOutputs()
{
    std::stringstream ss;

    for(int output_index=0; output_index < NrOfOutputs; output_index++) {
        ss << getDescriptionOfOutput(output_index) << std::endl;
    }

    return ss.str();
}

int DynamicRegressorGenerator::setRobotState(const KDL::JntArray &_q, const KDL::JntArray &_q_dot, const KDL::JntArray &_q_dotdot, const KDL::Twist& _base_velocity, const KDL::Twist& _base_acceleration)
{
    if( (int)_q.rows() != NrOfDOFs || (int)_q_dot.rows() != NrOfDOFs || (int)_q_dotdot.rows() != NrOfDOFs ) {
        if( verbose ) { std::cerr << "DynamicRegressorGenerator::setRobotState error: input size error" << std::endl; }
        return -1;
    }
    q = _q;
    dq = _q_dot;
    ddq = _q_dotdot;
    kinematic_base_velocity = _base_velocity;
    kinematic_base_acceleration = _base_acceleration;;
    return 0;
}

int DynamicRegressorGenerator::setRobotState(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const KDL::Twist& base_gravity)
{
    KDL::Twist dummy = KDL::Twist::Zero();
    return setRobotState(q,q_dot,q_dotdot,dummy,base_gravity);
}

int DynamicRegressorGenerator::setRobotStateAndSensors(const DynamicSample & sample)
{
    if( sample.getNrOfDOFs() != getNrOfDOFs() ) { return -1; }
    if( sample.getNrOfWrenchSensors() != getNrOfWrenchSensors() ) { return -2; }
    //if( sample.getNrOfTorqueSensors() != getNrOfTorqueSensors() ) { return -3; }

    q = sample.getJointPosition();
    dq = sample.getJointVelocity();
    ddq = sample.getJointAcceleration();
    kinematic_base_velocity = sample.getBaseVelocity();
    kinematic_base_acceleration = sample.getBaseSpatialAcceleration();

    for(int i=0; i < getNrOfWrenchSensors(); i++ ) {
        sensorMeasures.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,i,iDynTree::ToiDynTree(sample.getWrenchMeasure(i)));
    }

    /** \todo implement reading torque from sample, adding proper support for torque sensors */

    return 0;
}

int DynamicRegressorGenerator::computeRegressor( Eigen::MatrixXd & regressor, Eigen::VectorXd & known_terms)
{
    if( regressor.rows() != getNrOfOutputs() || regressor.cols() != getNrOfParameters() || known_terms.size() != getNrOfOutputs() )
    {
        if( verbose ) { std::cerr << "DynamicsRegressorGenerator::computeRegressor error: input size error" << std::endl; }
        return -1;
    }

    //Calculating the velocity and acceleration for each link
    rneaKinematicLoop(undirected_tree,q,dq,ddq,kinematic_traversal,kinematic_base_velocity,kinematic_base_acceleration,v,a);

    //Get the frame between each link and the base
#ifndef NDEBUG
    //if( verbose ) { std::cout << "DynamicsRegressorGenerator::computeRegressor computing regressor using as base frame: " << dynamic_traversal.getBaseLink()->getName() << std::endl; }
#endif
    getFramesLoop(undirected_tree,q,dynamic_traversal,X_dynamic_base);

    //Call specific regressors
    int start_row = 0;
    for(int i=0; i < (int)regressors_ptrs.size(); i++ ) {
        DynamicRegressorInterface * regr_ptr = regressors_ptrs[i];

        /**
         * Little workaround to avoid dynamic allocation of memory
         *
         * \todo this use of fixed buffers avoid this by using something similar
         *       to Eigen::Ref (unfortunately available only on Eigen 3.2.0)
         */

        switch( regr_ptr->getNrOfOutputs() )
        {
            case 6:
                if( consider_ft_offset ) {
                    assert(six_rows_buffer.cols() == this->getNrOfParameters());
                }
                regr_ptr->computeRegressor(q,dq,ddq,X_dynamic_base,v,a,sensorMeasures,measured_torques,six_rows_buffer,six_rows_vector);
                regressor.block(start_row,0,regr_ptr->getNrOfOutputs(),getNrOfParameters()) = six_rows_buffer;
                known_terms.segment(start_row,regr_ptr->getNrOfOutputs()) = six_rows_vector;
            break;
            case 1:
                regr_ptr->computeRegressor(q,dq,ddq,X_dynamic_base,v,a,sensorMeasures,measured_torques,one_rows_buffer,one_rows_vector);
                regressor.block(start_row,0,regr_ptr->getNrOfOutputs(),getNrOfParameters()) = one_rows_buffer;
                known_terms.segment(start_row,regr_ptr->getNrOfOutputs()) = one_rows_vector;
            break;
            default:
                //This should not be used,however a dynamic memory solution is added
                //as a fallback in case another type of regressor is added without modifyng this code
                Eigen::MatrixXd regr_buffer(regr_ptr->getNrOfOutputs(),getNrOfParameters());
                Eigen::VectorXd kt_buffer(regr_ptr->getNrOfOutputs());
                regr_ptr->computeRegressor(q,dq,ddq,X_dynamic_base,v,a,sensorMeasures,measured_torques,regr_buffer,kt_buffer);
                regressor.block(start_row,0,regr_ptr->getNrOfOutputs(),getNrOfParameters()) = regr_buffer;
                known_terms.segment(start_row,regr_ptr->getNrOfOutputs()) = kt_buffer;
            break;
        }

        start_row += regr_ptr->getNrOfOutputs();
    }

    return 0;

}

/*
class FloatingBaseRobotState
{
    public:
        JntArray q;
        JntArray dq;
        JntArray ddq;
}*/

int DynamicRegressorGenerator::generate_random_regressors(Eigen::MatrixXd & A,
                                                          const bool static_regressor,
                                                          const bool fixed_base,
                                                          const KDL::Vector grav_direction,
                                                          int n_samples,
                                                          const bool verbose)
{
    std::vector<int> dummy_int_vector;
    std::vector<double> dummy_double_vector;
    return generate_random_regressors(A,
                                      static_regressor,
                                      fixed_base,
                                      grav_direction,
                                      dummy_int_vector,
                                      dummy_double_vector,
                                      n_samples,
                                      verbose);
}


int DynamicRegressorGenerator::generate_random_regressors(Eigen::MatrixXd & A,
                                                          const bool static_regressor,
                                                          const bool fixed_base,
                                                          const KDL::Vector grav_direction,
                                                          std::vector<int> fixed_dofs,
                                                          std::vector<double> fixed_dofs_values,
                                                          int n_samples,
                                                          const bool verbose)
{
    if( n_samples < 0 ) return -1;
        int no = getNrOfOutputs();
        int np = getNrOfParameters();
        int nj = NrOfDOFs;


        //Generated robot state
        KDL::JntArray q(nj),dq(nj),ddq(nj);
        KDL::Twist a,v;

        //Convenience variables
        Eigen::MatrixXd regressor(no,np);
        Eigen::VectorXd kt(no);

        Eigen::MatrixXd V(np,np);
        Eigen::VectorXd sigma(np);

        if( A.rows() != np || A.cols() != np ) { A.resize(np,np); };

        /** \todo store and restore class state */

        for(int i=0; i < n_samples; i++ ) {
            //RegressorSolver
            //Excite with random data RegressorSolver (random q, dq, ddq (for energy, simply do not use ddq)
            /** \todo use limits of angles */
            q.data = M_PI*Eigen::VectorXd::Random(nj);
            if( static_regressor ) {
                SetToZero(dq);
                SetToZero(ddq);
            } else {
                dq.data = M_PI*Eigen::VectorXd::Random(nj);
                ddq.data = M_PI*Eigen::VectorXd::Random(nj);
            }

            for(int d=0; d < fixed_dofs.size(); d++ )
            {
                int fixed_dof = fixed_dofs[d];
                double fixed_dof_value = fixed_dofs_values[d];
                q(fixed_dof) = fixed_dof_value;
                dq(fixed_dof) = 0.0;
                ddq(fixed_dof) = 0.0;
            }

            if( fixed_base ) {
                //fixed base
                setRobotState(q,dq,ddq,KDL::Twist(grav_direction,KDL::Vector(0.0,0.0,0.0)));

            } else {
                //floating base
                Eigen::Map<Eigen::Vector3d>(a.vel.data) = M_PI*Eigen::Vector3d::Random();
                Eigen::Map<Eigen::Vector3d>(a.rot.data) = M_PI*Eigen::Vector3d::Random();
                Eigen::Map<Eigen::Vector3d>(v.vel.data) = M_PI*Eigen::Vector3d::Random();
                Eigen::Map<Eigen::Vector3d>(v.rot.data) = M_PI*Eigen::Vector3d::Random();

                if( static_regressor ) {
                    //In the case of the static regressor, only the acceleration (i.e. gravitational acceleration) is random
                    Eigen::Map<Eigen::Vector3d>(a.rot.data) = M_PI*Eigen::Vector3d::Zero();
                    Eigen::Map<Eigen::Vector3d>(v.vel.data) = M_PI*Eigen::Vector3d::Zero();
                    Eigen::Map<Eigen::Vector3d>(v.rot.data) = M_PI*Eigen::Vector3d::Zero();
                }


                setRobotState(q,dq,ddq,v,a);
            }

            int ret_value = computeRegressor(regressor,kt);
            if( ret_value != 0 ) { if( verbose ) std::cout <<  "computeIdentifiableSubspace fatal error" << std::endl; return -2; }

            if( i == 0 ) {
                A = regressor.transpose()*regressor;
            } else {
                A += regressor.transpose()*regressor;
            }
        }
        return 0;
}

int DynamicRegressorGenerator::computeNumericalIdentifiableSubspace(Eigen::MatrixXd & basis,
                                                                    const bool static_regressor,
                                                                    int n_samples,
                                                                    double tol,
                                                                    const bool verbose)
{
    bool fixed_base=false;
    KDL::Vector dummy_gravity;
    std::vector<int> dummy_int_vector;
    std::vector<double> dummy_double_vector;
    return computeNumericalIdentifiableSubspace(basis,
                                                static_regressor,
                                                fixed_base,
                                                dummy_gravity,
                                                dummy_int_vector,
                                                dummy_double_vector,
                                                n_samples,
                                                verbose);
}


int DynamicRegressorGenerator::computeNumericalIdentifiableSubspace(Eigen::MatrixXd & basis,
                                                                    const bool static_regressor,
                                                                    const bool fixed_base,
                                                                    const KDL::Vector grav_direction,
                                                                    const std::vector<int> fixed_dofs,
                                                                    const std::vector<double> fixed_dofs_values,
                                                                    double tol,
                                                                    int n_samples,
                                                                    bool verbose)
{
    if( fixed_dofs.size() != fixed_dofs_values.size() )
    {
        return -1;
    }
    Eigen::MatrixXd A(getNrOfParameters(),getNrOfParameters());
    //std::cout << "generate_random_regressors" << std::endl;
    generate_random_regressors(A,
                               static_regressor,
                               fixed_base,
                               grav_direction,
                               fixed_dofs,
                               fixed_dofs_values,
                               n_samples,
                               verbose);
    //std::cout << "generate_random_regressors" << std::endl;
    return getRowSpaceBasis(A,basis,-1.0,true);
}


int DynamicRegressorGenerator::computeSparseNumericalIdentifiableSubspaceV1(Eigen::MatrixXd & basis, const bool static_regressor, const bool fixed_base, const KDL::Vector grav_direction, double tol, int n_samples, bool verbose)
{

    if( n_samples < 0 ) { if(verbose) { std::cerr << "Error: number of samples of calculating the identifiable subspace cannot be negative" << std::endl; } return -1;  }

    int no = getNrOfOutputs();
    int np = getNrOfParameters();
    int nj = NrOfDOFs;

    //Generated robot state
    KDL::JntArray q(nj),dq(nj),ddq(nj);
    KDL::Twist a,v;


    Eigen::MatrixXd A(np,np); //working matrix
    Eigen::MatrixXd regressor(no,np);
    Eigen::VectorXd kt(no);

    Eigen::MatrixXd V(np,np);
    Eigen::VectorXd sigma(np);

    Eigen::MatrixXd dense_basis, sparse_basis;

    //Take track of the junction that come before the considered link
    std::vector<bool> considered_junctions(undirected_tree.getNrOfJunctions());

    //Calculate sequentially the subspace base, considering a subtree starting at a DOF at each loop
    for(int i=undirected_tree.getNrOfLinks()-1; i >=0; i-- ) {

        /////////////////////////////////////////
        /// Subspace till a given tree
        /////////////////////////////////////////
        KDL::CoDyCo::LinkMap::const_iterator link_it =  dynamic_traversal.getOrderedLink(i);

        #ifndef NDEBUG
        std::cerr << "Generating samples for all DOFs " << link_it->getName() << std::endl;
        #endif

        for(int sample=0; sample < n_samples; sample++ ) {
            //generated suitable state, exciting only the dofs before the current considered link

            //Initially set to zero all the coordinates, then generated random only for the dof before the considered link
            q.data = M_PI*Eigen::VectorXd::Random(nj);
            if( static_regressor ) {
                SetToZero(dq);
                SetToZero(ddq);
            } else {
                dq.data = M_PI*Eigen::VectorXd::Random(nj);
                ddq.data = M_PI*Eigen::VectorXd::Random(nj);
            }


            //Enumerate all the DOFs and junctions after the considered link
            //By default none of them
            fill(considered_junctions.begin(),considered_junctions.end(),false);
            std::deque<int> link_to_visit;
            link_to_visit.push_back(link_it->getLinkIndex());

            /** \todo move somewhere, undirected_tree ? */
            while(!link_to_visit.empty()) {
                int considered_link_index = link_to_visit.front();
                link_to_visit.pop_front();
                KDL::CoDyCo::LinkMap::const_iterator considered_link_it = undirected_tree.getLink(considered_link_index);

                for(int child=0; child < (int)considered_link_it->getNrOfAdjacentLinks(); child++ ) {
                    KDL::CoDyCo::JunctionMap::const_iterator considered_junction = considered_link_it->getAdjacentJoint(child);
                    KDL::CoDyCo::LinkMap::const_iterator considered_next_link = considered_link_it->getAdjacentLink(child);

                    if( considered_next_link == dynamic_traversal.getParentLink(considered_link_index) ) {
                        //This is the parent of the link, already considered
                        continue;
                    }

                    considered_junctions[considered_junction->getJunctionIndex()] = true;
                    link_to_visit.push_back(considered_next_link->getLinkIndex());
                }

            }


            //The base is always excited
            if( fixed_base ) {
                //fixed base
                setRobotState(q,dq,ddq,KDL::Twist(grav_direction,KDL::Vector(0.0,0.0,0.0)));

            } else {
                //floating base
                Eigen::Map<Eigen::Vector3d>(a.vel.data) = M_PI*Eigen::Vector3d::Random();
                Eigen::Map<Eigen::Vector3d>(a.rot.data) = M_PI*Eigen::Vector3d::Random();
                Eigen::Map<Eigen::Vector3d>(v.vel.data) = M_PI*Eigen::Vector3d::Random();
                Eigen::Map<Eigen::Vector3d>(v.rot.data) = M_PI*Eigen::Vector3d::Random();

                if( static_regressor ) {
                    //In the case of the static regressor, only the acceleration (i.e. gravitational acceleration) is random
                    SetToZero(a.rot);
                    SetToZero(v.vel);
                    SetToZero(v.rot);
                }

                setRobotState(q,dq,ddq,v,a);
            }

            int ret_value = computeRegressor(regressor,kt);
            if( ret_value != 0 ) { if( verbose ) std::cout <<  "computeSparseNumericalIdentifiableSubspace fatal error" << std::endl; return -2; }

            //For computing the sparse decomposition, we have to remove the line of the regressor of sensor after the considered link
            //Should be better to avoid computing this lines, but for now it easier to elimate them
            std::vector<bool> keep_regressor(regressors_ptrs.size());
            int reduced_regressor_rows = 0;
            for(int regr = 0; regr < (int)regressors_ptrs.size(); regr++ ) {
                bool keep_regressor_rows = false;
                DynamicRegressorInterface * considered_regr = regressors_ptrs[regr];

                std::vector<int> relative_junctions = considered_regr->getRelativeJunctions();

                if( relative_junctions.size() == 0 ) {
                    //The regressor is a global one and is not related to some specific junction
                    //So it is considered only when the last link is added
                    keep_regressor_rows = false;

                }

                if( relative_junctions.size() == 1 ) {
                    if( considered_junctions[relative_junctions[0]] ) {
                        keep_regressor_rows = true;
                    }

                } else {

                    for(int relative_junction_id = 0; relative_junction_id < (int)relative_junctions.size(); relative_junction_id++ ) {
                        int relative_junction = relative_junctions[relative_junction_id];
                        //If all the junction of the regressor are considered, keep the regressor
                        if( considered_junctions[relative_junction] ) {
                            keep_regressor_rows = true;
                        } else {
                           keep_regressor_rows = false;
                           break;
                        }
                    }
                }

                keep_regressor[regr] = keep_regressor_rows;

                if( keep_regressor_rows ) {
                    reduced_regressor_rows += considered_regr->getNrOfOutputs();
                }

            }

            Eigen::MatrixXd reduced_regressor = Eigen::MatrixXd(reduced_regressor_rows,regressor.cols());

            int reduced_regressor_rows_offset = 0;
            int regressor_rows_offset = 0;
            for(int regr = 0; regr < (int)regressors_ptrs.size(); regr++ ) {
                if( keep_regressor[regr] ) {
                    reduced_regressor.block(reduced_regressor_rows_offset,0,regressors_ptrs[regr]->getNrOfOutputs(),regressor.cols()) =
                        regressor.block(regressor_rows_offset,0,regressors_ptrs[regr]->getNrOfOutputs(),regressor.cols());

                    reduced_regressor_rows_offset += regressors_ptrs[regr]->getNrOfOutputs();
                }

                regressor_rows_offset += regressors_ptrs[regr]->getNrOfOutputs();
            }

            if( sample == 0 ) {
                A = reduced_regressor.transpose()*reduced_regressor;
            } else {
                A += reduced_regressor.transpose()*reduced_regressor;
            }
        }

        //std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~A~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
        getRowSpaceBasis(A,dense_basis);
        //std::cout<<"~~~~~~~~~~~~~~~~~~~~~obtained dense basis~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
        //std::cout << dense_basis << std::endl;

        if( i == (int)undirected_tree.getNrOfLinks()-1 ) {
            #ifndef NDEBUG
            std::cout << "Adding link " << link_it->getName() << " adds " << dense_basis.cols() << " base parameters" << std::endl;
            #endif
            sparse_basis = dense_basis;
        } else {
            /** \todo check if the dense basis is empty (fake base link) */
            //Update sparse_basis
            //Get the number of base parameters
            assert( (sparse_basis.rows() == dense_basis.rows()) || (sparse_basis.cols() == 0));
            assert( (sparse_basis.rows() == getNrOfParameters())  || (sparse_basis.cols() == 0));
            int old_nbp = sparse_basis.cols();


            //project the new dense_basis on the nullspace of the sparse_matrix
            Eigen::MatrixXd nullspace_dense_basis =  (Eigen::MatrixXd::Identity(np,np)-sparse_basis*sparse_basis.transpose())*dense_basis;

            Eigen::MatrixXd check_nullspace =  (Eigen::MatrixXd::Identity(np,np)-sparse_basis*sparse_basis.transpose())*sparse_basis;

            //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~sparse_basis~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            //std::cout << sparse_basis << std::endl;
            //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~null_space_dense_basis~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            //std::cout << nullspace_dense_basis << std::endl;
            //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~check_nullspace~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            //std::cout << check_nullspace << std::endl;

            //The rank of nullspace_dense_basis is then by definition new_nbp-old_nbp
            //It is then possible to get the row space basis of nullspace_dense_basis, and to add this vector to the sparse_basis
            Eigen::MatrixXd new_sparse_basis;

            std::cout << "~~~~~~~~~~~~~getRowSpaceBasis(nullspace_dense_basis,new_sparse_basis)~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            getRowSpaceBasis(nullspace_dense_basis.transpose(),new_sparse_basis);
            std::cout << "~~~~~~~~~~~~~~~new_sparse_basis~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            std::cout << new_sparse_basis << std::endl;

            int new_bp = new_sparse_basis.cols();

            #ifndef NDEBUG
            std::cout << "Adding link " << link_it->getName() << " adds " << new_bp << " base parameters for a total of " << old_nbp+new_bp << std::endl;
            #endif

            Eigen::MatrixXd enlarged_sparse_basis(np,old_nbp+new_bp);

            enlarged_sparse_basis.block(0,0,new_sparse_basis.rows(),new_sparse_basis.cols()) = new_sparse_basis;

            enlarged_sparse_basis.block(0,new_sparse_basis.cols(),sparse_basis.rows(),sparse_basis.cols()) = sparse_basis;

            sparse_basis = enlarged_sparse_basis;
        }

    }

    //Adding an iteration in which all the other regressor are considered
    {
        int old_nbp = sparse_basis.cols();

        Eigen::MatrixXd dense_basis;

        std::vector<int> dummy_int_vector;
        std::vector<double> dummy_double_vector;
        computeNumericalIdentifiableSubspace(dense_basis,
                                             static_regressor,
                                             fixed_base,
                                             grav_direction,
                                             dummy_int_vector,
                                             dummy_double_vector,
                                             tol,
                                             n_samples,
                                             verbose);

        Eigen::MatrixXd nullspace_dense_basis =  (Eigen::MatrixXd::Identity(np,np)-sparse_basis*sparse_basis.transpose())*dense_basis;

        //The rank of nullspace_dense_basis is then by definition new_nbp-old_nbp
        //It is then possible to get the row space basis of nullspace_dense_basis, and to add this vector to the sparse_basis
        Eigen::MatrixXd new_sparse_basis;

        std::cout << "~~~~~~~~~~~~~getRowSpaceBasis(nullspace_dense_basis,new_sparse_basis)~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            getRowSpaceBasis(nullspace_dense_basis.transpose(),new_sparse_basis);
            std::cout << "~~~~~~~~~~~~~~~new_sparse_basis~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            std::cout << new_sparse_basis << std::endl;

            int new_bp = new_sparse_basis.cols();

            #ifndef NDEBUG
            std::cout << "Adding all the other regressors adds " << new_bp << " base parameters for a total of " << old_nbp+new_bp << std::endl;
            #endif

            Eigen::MatrixXd enlarged_sparse_basis(np,old_nbp+new_bp);

            enlarged_sparse_basis.block(0,0,new_sparse_basis.rows(),new_sparse_basis.cols()) = new_sparse_basis;

            enlarged_sparse_basis.block(0,new_sparse_basis.cols(),sparse_basis.rows(),sparse_basis.cols()) = sparse_basis;

            sparse_basis = enlarged_sparse_basis;
    }

    basis = sparse_basis;

    std::cout << "SparseNumericalFinalTest" << std::endl;
    Eigen::MatrixXd dummy;
    getRowSpaceBasis(basis,dummy);


    return 0;
}

int DynamicRegressorGenerator::computeSparseNumericalIdentifiableSubspaceV2(Eigen::MatrixXd & basis, const bool static_regressor, const bool fixed_base, const KDL::Vector grav_direction, double tol, int n_samples, bool verbose)
{
    Eigen::MatrixXd A(getNrOfParameters(),getNrOfParameters());
    Eigen::MatrixXd sparseA(getNrOfParameters(),getNrOfParameters());

    Eigen::MatrixXd sparse_basis(getNrOfParameters(),0);
    Eigen::MatrixXd local_dense_basis;

    if( tol <= 0 ) {
        tol = 1e-7;
    }

    int np = getNrOfParameters();

    generate_random_regressors(A,static_regressor,fixed_base,grav_direction,n_samples,verbose);

    //For now do not support regressor with parameters different from the inertial one (f/t offset,friction,etc)
    assert(getNrOfParameters() == 10*NrOfRealLinks_gen);

    //Regroup from the leafs
    std::vector<bool> link_is_descendant(undirected_tree.getNrOfLinks(),false);

    for(int i = dynamic_traversal.getNrOfVisitedLinks()-1; i >=0 ; i-- ) {
        KDL::CoDyCo::LinkMap::const_iterator link_it =  dynamic_traversal.getOrderedLink(i);

        int link_index = link_it->getLinkIndex();

        int regressor_link_index = linkIndeces2regrColumns[link_index];

        if( regressor_link_index == -1 ) {
            //This link is fake link introduced in the structure to simulate multidof systems
            continue;
        }

        /**
         * \todo move somewhere
         *
         */
        //Check all the descendant of the links
        fill(link_is_descendant.begin(),link_is_descendant.end(),false);
        std::deque<int> link_to_visit;

        link_to_visit.push_back(link_it->getLinkIndex());

        while(!link_to_visit.empty()) {
                int considered_link_index = link_to_visit.front();
                link_to_visit.pop_front();
                KDL::CoDyCo::LinkMap::const_iterator considered_link_it = undirected_tree.getLink(considered_link_index);

                link_is_descendant[considered_link_index] = true;

                for(int child=0; child < (int)considered_link_it->getNrOfAdjacentLinks(); child++ ) {
                    KDL::CoDyCo::JunctionMap::const_iterator considered_junction = considered_link_it->getAdjacentJoint(child);
                    KDL::CoDyCo::LinkMap::const_iterator considered_next_link = considered_link_it->getAdjacentLink(child);

                    if( considered_next_link == dynamic_traversal.getParentLink(considered_link_index) ) {
                        //This is the parent of the link, already considered
                        continue;
                    }

                    link_to_visit.push_back(considered_next_link->getLinkIndex());
                }

         }

         sparseA.setZero();

         //Suboptimal
         /** \todo improve */
         for(int n = 0; n < (int)undirected_tree.getNrOfLinks(); n++ ) {
             if( !link_is_descendant[n] || linkIndeces2regrColumns[n] == -1 ) { continue; }
             for(int m = 0; m < (int)undirected_tree.getNrOfLinks(); m++ ) {
                if( !link_is_descendant[m] || linkIndeces2regrColumns[m] == -1) { continue; }

                sparseA.block<10,10>(10*linkIndeces2regrColumns[n],10*linkIndeces2regrColumns[m]) =
                    A.block<10,10>(10*linkIndeces2regrColumns[n],10*linkIndeces2regrColumns[m]);
             }
         }

         //std::cout << "sparseA" << std::endl;
         //std::cout << sparseA << std::endl;
         getRowSpaceBasis(sparseA,local_dense_basis);
         //std::cout << "local_dense_basis" << std::endl;
         //std::cout << local_dense_basis << std::endl;

         //Candidate basis (the sparse basis plus a full basis of the inertial parameters of this link
         Eigen::MatrixXd candidate_basis(np,sparse_basis.cols()+10);

         candidate_basis.setZero();

         candidate_basis.block(0,10,sparse_basis.rows(),sparse_basis.cols()) = sparse_basis;

         //std::cout << "candidate basis rows and cols " << candidate_basis.rows() << " " << candidate_basis.cols() << std::endl;
         //std::cout << "Requested block of " << 10*regressor_link_index << " " << candidate_basis.cols()-10 << std::endl;
         candidate_basis.block<10,10>(10*regressor_link_index,0) = Eigen::MatrixXd::Identity(10,10);

         //std::cout << "candidate basis " << std::endl;
         //std::cout << candidate_basis << std::endl;
         //Project the candidate on the nullspace of the local_dense_basis
         Eigen::MatrixXd candidate_basis_nullspace_projection = (Eigen::MatrixXd::Identity(getNrOfParameters(),getNrOfParameters())-local_dense_basis*local_dense_basis.transpose())*candidate_basis;
         //std::cout << "candidate basis nullspace projection" << std::endl;
         //std::cout << candidate_basis_nullspace_projection << std::endl;


         Eigen::MatrixXd surviving_basis(getNrOfParameters(),0);
         Eigen::MatrixXd basis_to_regroup(getNrOfParameters(),0);

         //Basis vector that have no component on the nullspace can remain in the sparse_basis, the other must be regrouped
         for(int l=0; l < candidate_basis_nullspace_projection.cols(); l++ ) {
             //std::cout << "column " << l << "has norm " << candidate_basis_nullspace_projection.block(0,l,np,1).norm() << std::endl;
             if( (candidate_basis_nullspace_projection.block(0,l,np,1)).norm() < tol ) {
                 surviving_basis.conservativeResize(surviving_basis.rows(),surviving_basis.cols()+1);
                 surviving_basis.block(0,surviving_basis.cols()-1,np,1) = candidate_basis.block(0,l,np,1);
             } else {
                 basis_to_regroup.conservativeResize(basis_to_regroup.rows(),basis_to_regroup.cols()+1);
                 basis_to_regroup.block(0,basis_to_regroup.cols()-1,np,1) = candidate_basis.block(0,l,np,1);
             }
         }

         Eigen::MatrixXd regrouped_basis;
         getRowSpaceBasis((local_dense_basis*local_dense_basis.transpose()*basis_to_regroup).transpose(),regrouped_basis,1e-5,true);

         /*
         std::cout << "surviving_basis " << std::endl;
         std::cout << surviving_basis << std::endl;

         std::cout << "basis_to_regroup " << std::endl;
         std::cout << basis_to_regroup << std::endl;
         std::cout << "regrouped_basis" << std::endl;
         std::cout << regrouped_basis << std::endl;
         */

         sparse_basis = surviving_basis;

         sparse_basis.conservativeResize(sparse_basis.rows(),sparse_basis.cols()+regrouped_basis.cols());

         sparse_basis.block(0,sparse_basis.cols()-regrouped_basis.cols(),regrouped_basis.rows(),regrouped_basis.cols()) = regrouped_basis;

         std::cout << "After adding link " << link_it->getName() << " we have " << sparse_basis.cols() << "base parameters" << std::endl;
         std::cout << "\tCandidate: " << candidate_basis.cols() << " surviving: " << surviving_basis.cols() << " basis to regroup " << basis_to_regroup.cols() << " regrouped " << regrouped_basis.cols() << std::endl;
         //std::cout << "obtained sparse_basis " << std::endl;
         //std::cout << sparse_basis << std::endl;
         //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    }

    basis = sparse_basis;

    return 0;
}

int DynamicRegressorGenerator::computeSparseNumericalIdentifiableSubspaceSimpleAlgorithm(Eigen::MatrixXd & basis, const bool static_regressor, const bool fixed_base, const KDL::Vector grav_direction, double tol, int n_samples, bool verbose)
{
    Eigen::MatrixXd A(getNrOfParameters(),getNrOfParameters());
    Eigen::MatrixXd sparseA(getNrOfParameters(),getNrOfParameters());

    Eigen::MatrixXd sparse_basis(getNrOfParameters(),0);
    Eigen::MatrixXd param_to_regroup(getNrOfParameters(),0);

    Eigen::MatrixXd complete_identifiable_space_dense_basis;

    if( tol <= 0 ) {
        tol = 1e-7;
    }

    int np = getNrOfParameters();

    generate_random_regressors(A,static_regressor,fixed_base,grav_direction,n_samples,verbose);

    //For now do not support regressor with parameters different from the inertial one (f/t offset,friction,etc)
    assert(getNrOfParameters() == 10*NrOfRealLinks_gen);

   getRowSpaceBasis(A,complete_identifiable_space_dense_basis);

   Eigen::MatrixXd identifiable_subspace_projector = complete_identifiable_space_dense_basis*complete_identifiable_space_dense_basis.transpose();
   Eigen::MatrixXd nonidentifiable_subspace_projector = Eigen::MatrixXd::Identity(np,np)-identifiable_subspace_projector;

    for(int i = dynamic_traversal.getNrOfVisitedLinks()-1; i >=0 ; i-- ) {
        KDL::CoDyCo::LinkMap::const_iterator link_it =  dynamic_traversal.getOrderedLink(i);

        int link_index = link_it->getLinkIndex();

        int regressor_link_index = linkIndeces2regrColumns[link_index];

        if( regressor_link_index == -1 ) {
            //This link is fake link introduced in the structure to simulate multidof systems
            continue;
        }

        //First we check if any local parameter is completly identifiable
        Eigen::MatrixXd local_parameters(np,10);

        local_parameters.setZero();
        local_parameters.block<10,10>(10*regressor_link_index,0) = Eigen::MatrixXd::Identity(10,10);

        param_to_regroup.conservativeResize(np,param_to_regroup.cols()+local_parameters.cols());
        param_to_regroup.block(0,param_to_regroup.cols()-local_parameters.cols(),np,local_parameters.cols()) = local_parameters;

        Eigen::MatrixXd param_to_regroup_identifiable_projections = identifiable_subspace_projector*param_to_regroup;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(param_to_regroup_identifiable_projections.transpose(), Eigen::ComputeThinU | Eigen::ComputeFullV);

         Eigen::VectorXd sigma = svd.singularValues();

         Eigen::MatrixXd V = svd.matrixV();

         //std::cout << "sigma " << std::endl << sigma << std::endl;

         //The columns of have that have singular value 1 are the regrouped paramaters till this link
         for(int l=0; l < sigma.size(); l++ ) {
             if( fabs(1-sigma[l]) < tol ) {
                 sparse_basis.conservativeResize(np,sparse_basis.cols()+1);
                 sparse_basis.block(0,sparse_basis.cols()-1,np,1) = V.block(0,l,np,1);
             }
         }

         //std::cout << "getRowSpaceBasis of parameter_to_regroup projected on current sparse_basis nullspace" << std::endl;
         getRowSpaceBasis(((Eigen::MatrixXd::Identity(np,np)-sparse_basis*sparse_basis.transpose())*param_to_regroup).transpose(),
                          param_to_regroup);

         /*
         std::cout << "sparse_basis " << std::endl;
         std::cout << sparse_basis << std::endl;

         std::cout << "parameters to regroup 2 " << std::endl;
         std::cout << parameters_to_regroup << std::endl;
         */

         //std::cout << "After adding link " << link_it->getName() << " we have " << sparse_basis.cols() << "base parameters" << std::endl;
         //std::cout << "\tTo regroup : " << param_to_regroup.cols() << std::endl;
         //std::cout << sparse_basis << std::endl;
         //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    }

    std::cout << "Sparse basis cols: " << sparse_basis.cols() << " other " << complete_identifiable_space_dense_basis.cols() << std::endl;
    assert(sparse_basis.cols() == complete_identifiable_space_dense_basis.cols());

    basis = sparse_basis;

    //Reverting order of columns for a more reasonable result
    for(int col=0; col < basis.cols(); col++ ) {
        basis.block(0,col,np,1) = sparse_basis.block(0,sparse_basis.cols()-1-col,np,1);
    }

    return 0;
}

int DynamicRegressorGenerator::computeSparseNumericalIdentifiableSubspaceAdvancedAlgorithm(Eigen::MatrixXd & basis, const bool static_regressor, const bool fixed_base, const KDL::Vector grav_direction, double tol, int n_samples, bool verbose)
{
    Eigen::MatrixXd A(getNrOfParameters(),getNrOfParameters());
    Eigen::MatrixXd sparseA(getNrOfParameters(),getNrOfParameters());

    Eigen::MatrixXd sparse_basis(getNrOfParameters(),0);
    Eigen::MatrixXd param_to_regroup(getNrOfParameters(),0);

    Eigen::MatrixXd complete_identifiable_space_dense_basis;

    if( tol <= 0 ) {
        tol = 1e-7;
    }

    int np = getNrOfParameters();

    generate_random_regressors(A,static_regressor,fixed_base,grav_direction,n_samples,verbose);

    //For now do not support regressor with parameters different from the inertial one (f/t offset,friction,etc)
    assert(getNrOfParameters() == 10*NrOfRealLinks_gen);

   getRowSpaceBasis(A,complete_identifiable_space_dense_basis);

   Eigen::MatrixXd identifiable_subspace_projector = complete_identifiable_space_dense_basis*complete_identifiable_space_dense_basis.transpose();
   Eigen::MatrixXd nonidentifiable_subspace_projector = Eigen::MatrixXd::Identity(np,np)-identifiable_subspace_projector;

    for(int i = dynamic_traversal.getNrOfVisitedLinks()-1; i >=0 ; i-- ) {
        KDL::CoDyCo::LinkMap::const_iterator link_it =  dynamic_traversal.getOrderedLink(i);

        int link_index = link_it->getLinkIndex();

        int regressor_link_index = linkIndeces2regrColumns[link_index];

        if( regressor_link_index == -1 ) {
            //This link is fake link introduced in the structure to simulate multidof systems
            continue;
        }

        //First we check if any local parameter is completly identifiable
        Eigen::MatrixXd local_parameters(np,10);
        Eigen::MatrixXd local_parameter_partially_identifiable(np,0);

        local_parameters.setZero();
        local_parameters.block<10,10>(10*regressor_link_index,0) = Eigen::MatrixXd::Identity(10,10);

        Eigen::MatrixXd local_parameters_identifiable_projections = identifiable_subspace_projector*local_parameters;

        //Try to put the individual parameters as a base parameter
         for(int l=0; l < local_parameters_identifiable_projections.cols(); l++ ) {
             //std::cout << "column " << l << "has norm " << candidate_basis_nullspace_projection.block(0,l,np,1).norm() << std::endl;
             if( fabs(1-(local_parameters_identifiable_projections.block(0,l,np,1)).norm()) < tol ) {
                 sparse_basis.conservativeResize(sparse_basis.rows(),sparse_basis.cols()+1);
                 sparse_basis.block(0,sparse_basis.cols()-1,np,1) = local_parameters.block(0,l,np,1);
             } else {
                 local_parameter_partially_identifiable.conservativeResize(local_parameter_partially_identifiable.rows(),local_parameter_partially_identifiable.cols()+1);
                 local_parameter_partially_identifiable.block(0,local_parameter_partially_identifiable.cols()-1,np,1) = local_parameters.block(0,l,np,1);
             }
        }

        //
        Eigen::MatrixXd local_parameter_partially_identifiable_identifiable_projections = identifiable_subspace_projector*local_parameter_partially_identifiable;

        Eigen::MatrixXd new_param_to_regroup(np,0);

        Eigen::JacobiSVD<Eigen::MatrixXd> svd_local(local_parameter_partially_identifiable_identifiable_projections.transpose(), Eigen::ComputeThinU | Eigen::ComputeFullV);

        Eigen::VectorXd sigma_local = svd_local.singularValues();

        Eigen::MatrixXd V_local = svd_local.matrixV();

        //std::cout << "sigma_local " << std::endl << sigma_local << std::endl;

        //The columns of have that have singular value 1 are the regrouped paramaters till this link
        for(int l=0; l < sigma_local.size(); l++ ) {
            if( fabs(1-sigma_local[l]) < tol ) {
                 sparse_basis.conservativeResize(np,sparse_basis.cols()+1);
                 sparse_basis.block(0,sparse_basis.cols()-1,np,1) = V_local.block(0,l,np,1);
            }
        }

         //std::cout << "getRowSpaceBasis of parameter_to_regroup projected on current sparse_basis nullspace" << std::endl;
         getRowSpaceBasis(((Eigen::MatrixXd::Identity(np,np)-sparse_basis*sparse_basis.transpose())*local_parameter_partially_identifiable).transpose(),
                          new_param_to_regroup);

        param_to_regroup.conservativeResize(np,param_to_regroup.cols()+new_param_to_regroup.cols());
        param_to_regroup.block(0,param_to_regroup.cols()-new_param_to_regroup.cols(),np,new_param_to_regroup.cols()) = new_param_to_regroup;

        //Remove if not working


        /*
        param_to_regroup.conservativeResize(np,param_to_regroup.cols()+local_parameter_partially_identifiable.cols());
        param_to_regroup.block(0,param_to_regroup.cols()-local_parameter_partially_identifiable.cols(),np,local_parameter_partially_identifiable.cols()) = local_parameter_partially_identifiable;
        */


        Eigen::MatrixXd param_to_regroup_identifiable_projections = identifiable_subspace_projector*param_to_regroup;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(param_to_regroup_identifiable_projections.transpose(), Eigen::ComputeThinU | Eigen::ComputeFullV);

         Eigen::VectorXd sigma = svd.singularValues();

         Eigen::MatrixXd V = svd.matrixV();

         //std::cout << "sigma " << std::endl << sigma << std::endl;

         //The columns of have that have singular value 1 are the regrouped paramaters till this link
         for(int l=0; l < sigma.size(); l++ ) {
             if( fabs(1-sigma[l]) < tol ) {
                 sparse_basis.conservativeResize(np,sparse_basis.cols()+1);
                 sparse_basis.block(0,sparse_basis.cols()-1,np,1) = V.block(0,l,np,1);
             }
         }

         //std::cout << "getRowSpaceBasis of parameter_to_regroup projected on current sparse_basis nullspace" << std::endl;
         getRowSpaceBasis(((Eigen::MatrixXd::Identity(np,np)-sparse_basis*sparse_basis.transpose())*param_to_regroup).transpose(),
                          param_to_regroup);

         /*
         std::cout << "sparse_basis " << std::endl;
         std::cout << sparse_basis << std::endl;

         std::cout << "parameters to regroup 2 " << std::endl;
         std::cout << parameters_to_regroup << std::endl;
         */

         //std::cout << "After adding link " << link_it->getName() << " we have " << sparse_basis.cols() << "base parameters" << std::endl;
         //std::cout << "\tTo regroup : " << param_to_regroup.cols() << std::endl;
         //std::cout << sparse_basis << std::endl;
         //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    }

    std::cout << "Sparse basis cols: " << sparse_basis.cols() << " other " << complete_identifiable_space_dense_basis.cols() << std::endl;
    assert(sparse_basis.cols() == complete_identifiable_space_dense_basis.cols());

    basis = sparse_basis;

    //Reverting order of columns for a more reasonable result
    for(int col=0; col < basis.cols(); col++ ) {
        basis.block(0,col,np,1) = sparse_basis.block(0,sparse_basis.cols()-1-col,np,1);
    }

    return 0;
}


int DynamicRegressorGenerator::computeSparseNumericalIdentifiableSubspaceAdvancedPaper(Eigen::MatrixXd & basis, const bool static_regressor, const bool fixed_base, const KDL::Vector grav_direction, double tol, int n_samples, bool verbose)
{
    Eigen::MatrixXd A(getNrOfParameters(),getNrOfParameters());

    Eigen::MatrixXd sparse_basis_Z_s(getNrOfParameters(),0);

    Eigen::MatrixXd new_sparse_basis;

    Eigen::MatrixXd dense_basis;

    Eigen::MatrixXd constraint_matrix(3*getNrOfParameters(),getNrOfParameters());

    Eigen::MatrixXd projector_B_nu_i(getNrOfParameters(),getNrOfParameters());

    Eigen::MatrixXd local_parameter_basis_Z_Bi(getNrOfParameters(),10);

    //Regroup from the leafs
    std::vector<bool> link_is_descendant(undirected_tree.getNrOfLinks(),false);

    int np = getNrOfParameters();

    generate_random_regressors(A,static_regressor,fixed_base,grav_direction,n_samples,verbose);

    //For now do not support regressor with parameters different from the inertial one (f/t offset,friction,etc)
    assert(getNrOfParameters() == 10*NrOfRealLinks_gen);

    getRowSpaceBasis(A,dense_basis);

    Eigen::MatrixXd identifiable_subspace_projector = dense_basis*dense_basis.transpose();
    Eigen::MatrixXd nonidentifiable_subspace_projector = Eigen::MatrixXd::Identity(np,np)-identifiable_subspace_projector;

    for(int i = dynamic_traversal.getNrOfVisitedLinks()-1; i >=0 ; i-- ) {
        KDL::CoDyCo::LinkMap::const_iterator link_it =  dynamic_traversal.getOrderedLink(i);

        int link_index = link_it->getLinkIndex();

        int regressor_link_index = linkIndeces2regrColumns[link_index];

        if( regressor_link_index == -1 ) {
            //This link is fake link introduced in the structure to simulate multidof systems
            continue;
        }

        ////////////////////////////////////////////
        //// ADDING TYPE III BASE PARAMETERS BASIS
        /////////////////////////////////////////////
        double tol_III;
        if( tol <= 0.0 ) {
            tol_III = 1e-7;
        } else {
            tol_III = tol;
        }


        local_parameter_basis_Z_Bi.setZero();
        local_parameter_basis_Z_Bi.block<10,10>(10*regressor_link_index,0) = Eigen::MatrixXd::Identity(10,10);

        Eigen::MatrixXd local_parameters_identifiable_projections = identifiable_subspace_projector*local_parameter_basis_Z_Bi;

        //Try to put the individual parameters as a base parameter
        int added_param = 0;
         for(int l=0; l < local_parameters_identifiable_projections.cols(); l++ ) {
             //std::cout << "column " << l << "has norm " << candidate_basis_nullspace_projection.block(0,l,np,1).norm() << std::endl;
             if( fabs(1-(local_parameters_identifiable_projections.block(0,l,np,1)).norm()) < tol_III ) {
                 sparse_basis_Z_s.conservativeResize(sparse_basis_Z_s.rows(),sparse_basis_Z_s.cols()+1);
                 sparse_basis_Z_s.block(0,sparse_basis_Z_s.cols()-1,np,1) = local_parameter_basis_Z_Bi.block(0,l,np,1);
                 added_param++;
             }
        }

        #ifndef NDEBUG
        //std::cout << "Added " << added_param << " type III base parameters relative to link " << link_index << std::endl;
        #endif


        ////////////////////////////////////////////////////////
        //// ADDING TYPE II BASE PARAMETERS BASIS
        //////////////////////////////////////////////////////////

        //Building constraint matrix
        assert(constraint_matrix.rows() == 3*np);

        constraint_matrix.setZero();
        constraint_matrix.block(0,0,np,np) = nonidentifiable_subspace_projector;
        constraint_matrix.block(np,0,np,np) = Eigen::MatrixXd::Identity(np,np)-local_parameter_basis_Z_Bi*local_parameter_basis_Z_Bi.transpose();
        constraint_matrix.block(2*np,0,np,np) = sparse_basis_Z_s*sparse_basis_Z_s.transpose();

        getKernelSpaceBasis(constraint_matrix,new_sparse_basis,tol);

        sparse_basis_Z_s.conservativeResize(sparse_basis_Z_s.rows(),sparse_basis_Z_s.cols()+new_sparse_basis.cols());
        sparse_basis_Z_s.block(0,sparse_basis_Z_s.cols()-new_sparse_basis.cols(),np,new_sparse_basis.cols()) = new_sparse_basis;


        #ifndef NDEBUG
        //std::cout << "Constraint matrix " << std::endl;
        //std::cout << constraint_matrix << std::endl;
        //std::cout << "Added " << new_sparse_basis.cols() << " type II base parameters relative to link " << link_index << std::endl;
        #endif


        ////////////////////////////////////////////////////////
        //// ADDING TYPE I BASE PARAMETERS BASIS
        //////////////////////////////////////////////////////////

        //Building a basis of the inertial parameters of the subtree
        //For convenience we directly build the subtree basis projector

        //Move the "find links in a subtree" logic somewhere else \todo
        fill(link_is_descendant.begin(),link_is_descendant.end(),false);
        std::deque<int> link_to_visit;

        link_to_visit.push_back(link_it->getLinkIndex());

        while(!link_to_visit.empty()) {
                int considered_link_index = link_to_visit.front();
                link_to_visit.pop_front();
                KDL::CoDyCo::LinkMap::const_iterator considered_link_it = undirected_tree.getLink(considered_link_index);

                link_is_descendant[considered_link_index] = true;

                for(int child=0; child < (int)considered_link_it->getNrOfAdjacentLinks(); child++ ) {
                    KDL::CoDyCo::JunctionMap::const_iterator considered_junction = considered_link_it->getAdjacentJoint(child);
                    KDL::CoDyCo::LinkMap::const_iterator considered_next_link = considered_link_it->getAdjacentLink(child);

                    if( considered_next_link == dynamic_traversal.getParentLink(considered_link_index) ) {
                        //This is the parent of the link, already considered
                        continue;
                    }

                    link_to_visit.push_back(considered_next_link->getLinkIndex());
                }

         }

         projector_B_nu_i.setZero();

         //Suboptimal
         /** \todo improve */
         for(int n = 0; n < (int)undirected_tree.getNrOfLinks(); n++ ) {
             if( !link_is_descendant[n] || linkIndeces2regrColumns[n] == -1 ) { continue; }

              projector_B_nu_i.block<10,10>(10*linkIndeces2regrColumns[n],10*linkIndeces2regrColumns[n]) = Eigen::Matrix<double,10,10>::Identity();

         }

        //Building constraint matrix
        assert(constraint_matrix.rows() == 3*np);

        constraint_matrix.block(0,0,np,np) = nonidentifiable_subspace_projector;
        constraint_matrix.block(np,0,np,np) = Eigen::MatrixXd::Identity(np,np)-projector_B_nu_i;
        constraint_matrix.block(2*np,0,np,np) = sparse_basis_Z_s*sparse_basis_Z_s.transpose();

        //Calculate kernel with a SVD decomposition
        getKernelSpaceBasis(constraint_matrix,new_sparse_basis);

        sparse_basis_Z_s.conservativeResize(sparse_basis_Z_s.rows(),sparse_basis_Z_s.cols()+new_sparse_basis.cols());
        sparse_basis_Z_s.block(0,sparse_basis_Z_s.cols()-new_sparse_basis.cols(),np,new_sparse_basis.cols()) = new_sparse_basis;


        #ifndef NDEBUG
        //std::cout << " projector_B_nu_i " << std::endl;
        //std::cout << projector_B_nu_i << std::endl;
        //std::cout << "Constraint matrix " << std::endl;
        //std::cout << constraint_matrix << std::endl;
        //std::cout << "Added " << new_sparse_basis.cols() << " type I base parameters relative to link " << link_index << std::endl;
        #endif

        #ifndef NDEBUG
        //std::cout << "Considered link " << link_index << std::endl;
        #endif
        //std::cout << "Considered link " << link_index << std::endl;



    }

    std::cout << "Sparse basis cols: " << sparse_basis_Z_s.cols() << " other " << dense_basis.cols() << std::endl;
    assert(sparse_basis_Z_s.cols() == dense_basis.cols());

    basis = sparse_basis_Z_s;

    //Reverting order of columns for a more reasonable result
    for(int col=0; col < basis.cols(); col++ ) {
        basis.block(0,col,np,1) = sparse_basis_Z_s.block(0,sparse_basis_Z_s.cols()-1-col,np,1);
    }

    return 0;
}

int DynamicRegressorGenerator::computeSparseNumericalIdentifiableSubspaceSimplePaper(Eigen::MatrixXd & basis, const bool static_regressor, const bool fixed_base, const KDL::Vector grav_direction, double tol, int n_samples, bool verbose)
{
    Eigen::MatrixXd A(getNrOfParameters(),getNrOfParameters());

    Eigen::MatrixXd sparse_basis_Z_s(getNrOfParameters(),0);

    Eigen::MatrixXd new_sparse_basis;

    Eigen::MatrixXd dense_basis;


    Eigen::MatrixXd constraint_matrix(3*getNrOfParameters(),getNrOfParameters());

    Eigen::MatrixXd projector_B_nu_i(getNrOfParameters(),getNrOfParameters());

    Eigen::MatrixXd local_parameter_basis_Z_Bi(getNrOfParameters(),10);

    //Regroup from the leafs
    std::vector<bool> link_is_descendant(undirected_tree.getNrOfLinks(),false);

    int np = getNrOfParameters();

    generate_random_regressors(A,static_regressor,fixed_base,grav_direction,n_samples,verbose);

    //For now do not support regressor with parameters different from the inertial one (f/t offset,friction,etc)
    assert(getNrOfParameters() == 10*NrOfRealLinks_gen);

    getRowSpaceBasis(A,dense_basis);

    Eigen::MatrixXd identifiable_subspace_projector = dense_basis*dense_basis.transpose();
    Eigen::MatrixXd nonidentifiable_subspace_projector = Eigen::MatrixXd::Identity(np,np)-identifiable_subspace_projector;

    for(int i = dynamic_traversal.getNrOfVisitedLinks()-1; i >=0 ; i-- ) {
        KDL::CoDyCo::LinkMap::const_iterator link_it =  dynamic_traversal.getOrderedLink(i);

        int link_index = link_it->getLinkIndex();

        int regressor_link_index = linkIndeces2regrColumns[link_index];

        if( regressor_link_index == -1 ) {
            //This link is fake link introduced in the structure to simulate multidof systems
            continue;
        }

        ////////////////////////////////////////////
        //// ADDING TYPE III BASE PARAMETERS BASIS
        /////////////////////////////////////////////
        /*
        double tol_III;
        if( tol <= 0.0 ) {
            tol_III = 1e-7;
        } else {
            tol_III = tol;
        }*/

        ////////////////////////////////////////////////////////
        //// ADDING TYPE I BASE PARAMETERS BASIS
        //////////////////////////////////////////////////////////

        //Building a basis of the inertial parameters of the subtree
        //For convenience we directly build the subtree basis projector

        //Move the "find links in a subtree" logic somewhere else \todo
        fill(link_is_descendant.begin(),link_is_descendant.end(),false);
        std::deque<int> link_to_visit;

        link_to_visit.push_back(link_it->getLinkIndex());

        while(!link_to_visit.empty()) {
                int considered_link_index = link_to_visit.front();
                link_to_visit.pop_front();
                KDL::CoDyCo::LinkMap::const_iterator considered_link_it = undirected_tree.getLink(considered_link_index);

                link_is_descendant[considered_link_index] = true;

                for(int child=0; child < (int)considered_link_it->getNrOfAdjacentLinks(); child++ ) {
                    KDL::CoDyCo::JunctionMap::const_iterator considered_junction = considered_link_it->getAdjacentJoint(child);
                    KDL::CoDyCo::LinkMap::const_iterator considered_next_link = considered_link_it->getAdjacentLink(child);

                    if( considered_next_link == dynamic_traversal.getParentLink(considered_link_index) ) {
                        //This is the parent of the link, already considered
                        continue;
                    }

                    link_to_visit.push_back(considered_next_link->getLinkIndex());
                }

         }

         projector_B_nu_i.setZero();

         //Suboptimal
         /** \todo improve */
         for(int n = 0; n < (int)undirected_tree.getNrOfLinks(); n++ ) {
             if( !link_is_descendant[n] || linkIndeces2regrColumns[n] == -1 ) { continue; }

              projector_B_nu_i.block<10,10>(10*linkIndeces2regrColumns[n],10*linkIndeces2regrColumns[n]) = Eigen::Matrix<double,10,10>::Identity();

         }

        //Building constraint matrix
        assert(constraint_matrix.rows() == 3*np);

        constraint_matrix.block(0,0,np,np) = nonidentifiable_subspace_projector;
        constraint_matrix.block(np,0,np,np) = Eigen::MatrixXd::Identity(np,np)-projector_B_nu_i;
        constraint_matrix.block(2*np,0,np,np) = sparse_basis_Z_s*sparse_basis_Z_s.transpose();

        //Calculate kernel with a SVD decomposition
        getKernelSpaceBasis(constraint_matrix,new_sparse_basis);

        sparse_basis_Z_s.conservativeResize(sparse_basis_Z_s.rows(),sparse_basis_Z_s.cols()+new_sparse_basis.cols());
        sparse_basis_Z_s.block(0,sparse_basis_Z_s.cols()-new_sparse_basis.cols(),np,new_sparse_basis.cols()) = new_sparse_basis;


        #ifndef NDEBUG
        //std::cout << " projector_B_nu_i " << std::endl;
        //std::cout << projector_B_nu_i << std::endl;
        //std::cout << "Constraint matrix " << std::endl;
        //std::cout << constraint_matrix << std::endl;
        //std::cout << "Added " << new_sparse_basis.cols() << " type I base parameters relative to link " << link_index << std::endl;
        #endif

        #ifndef NDEBUG
        //std::cout << "Considered link " << link_index << std::endl;
        #endif
        //std::cout << "Considered link " << link_index << std::endl;



    }

    std::cout << "Sparse basis cols: " << sparse_basis_Z_s.cols() << " other " << dense_basis.cols() << std::endl;
    assert(sparse_basis_Z_s.cols() == dense_basis.cols());

    basis = sparse_basis_Z_s;

    //Reverting order of columns for a more reasonable result
    for(int col=0; col < basis.cols(); col++ ) {
        basis.block(0,col,np,1) = sparse_basis_Z_s.block(0,sparse_basis_Z_s.cols()-1-col,np,1);
    }

    return 0;
}


int DynamicRegressorGenerator::computeSparseNumericalIdentifiableSubspaceSimpleGolub(Eigen::MatrixXd & basis, const bool static_regressor, const bool fixed_base, const KDL::Vector grav_direction, double tol, int n_samples, bool verbose)
{
    Eigen::MatrixXd A(getNrOfParameters(),getNrOfParameters());

    Eigen::MatrixXd sparse_basis_Z_s(getNrOfParameters(),0);

    Eigen::MatrixXd new_sparse_basis;

    Eigen::MatrixXd dense_basis;

    Eigen::MatrixXd B_nu_i(getNrOfParameters(),0);

    Eigen::MatrixXd local_parameter_basis_Z_Bi(getNrOfParameters(),10);

    //Regroup from the leafs
    std::vector<bool> link_is_descendant(undirected_tree.getNrOfLinks(),false);

    int np = getNrOfParameters();

    generate_random_regressors(A,static_regressor,fixed_base,grav_direction,n_samples,verbose);

    //For now do not support regressor with parameters different from the inertial one (f/t offset,friction,etc)
    assert(getNrOfParameters() == 10*NrOfRealLinks_gen);

    getRowSpaceBasis(A,dense_basis);


    for(int i = dynamic_traversal.getNrOfVisitedLinks()-1; i >=0 ; i-- ) {
        KDL::CoDyCo::LinkMap::const_iterator link_it =  dynamic_traversal.getOrderedLink(i);

        int link_index = link_it->getLinkIndex();

        int regressor_link_index = linkIndeces2regrColumns[link_index];

        if( regressor_link_index == -1 ) {
            //This link is fake link introduced in the structure to simulate multidof systems
            continue;
        }

        ////////////////////////////////////////////
        //// ADDING TYPE III BASE PARAMETERS BASIS
        /////////////////////////////////////////////
        /*
        double tol_III;
        if( tol <= 0.0 ) {
            tol_III = 1e-7;
        } else {
            tol_III = tol;
        }*/

        ////////////////////////////////////////////////////////
        //// ADDING TYPE I BASE PARAMETERS BASIS
        //////////////////////////////////////////////////////////

        //Building a basis of the inertial parameters of the subtree
        //For convenience we directly build the subtree basis projector

        //Move the "find links in a subtree" logic somewhere else \todo
        fill(link_is_descendant.begin(),link_is_descendant.end(),false);
        std::deque<int> link_to_visit;

        link_to_visit.push_back(link_it->getLinkIndex());

        int B_nu_i_size = 0;
        while(!link_to_visit.empty()) {
                int considered_link_index = link_to_visit.front();
                link_to_visit.pop_front();
                KDL::CoDyCo::LinkMap::const_iterator considered_link_it = undirected_tree.getLink(considered_link_index);

                link_is_descendant[considered_link_index] = true;
                B_nu_i_size += 10;

                for(int child=0; child < (int)considered_link_it->getNrOfAdjacentLinks(); child++ ) {
                    KDL::CoDyCo::JunctionMap::const_iterator considered_junction = considered_link_it->getAdjacentJoint(child);
                    KDL::CoDyCo::LinkMap::const_iterator considered_next_link = considered_link_it->getAdjacentLink(child);

                    if( considered_next_link == dynamic_traversal.getParentLink(considered_link_index) ) {
                        //This is the parent of the link, already considered
                        continue;
                    }

                    link_to_visit.push_back(considered_next_link->getLinkIndex());
                }

         }

        B_nu_i.resize(getNrOfParameters(),B_nu_i_size);
        B_nu_i.setZero();


         //Suboptimal
         /** \todo improve */
         int free_column = 0;
         for(int n = 0; n < (int)undirected_tree.getNrOfLinks(); n++ ) {
             if( !link_is_descendant[n] || linkIndeces2regrColumns[n] == -1 ) { continue; }

              B_nu_i.block<10,10>(10*linkIndeces2regrColumns[n],free_column) = Eigen::Matrix<double,10,10>::Identity();
              free_column+= 10;
         }

         //std::cout << "B_nu_i " << std::endl;
         //std::cout << B_nu_i << std::endl;

        //Building constraint matrix
        /*
        assert(constraint_matrix.rows() == 3*np);

        constraint_matrix.block(0,0,np,np) = nonidentifiable_subspace_projector;
        constraint_matrix.block(np,0,np,np) = Eigen::MatrixXd::Identity(np,np)-projector_B_nu_i;
        constraint_matrix.block(2*np,0,np,np) = sparse_basis_Z_s*sparse_basis_Z_s.transpose();

        //Calculate kernel with a SVD decomposition
        getKernelSpaceBasis(constraint_matrix,new_sparse_basis);
        */
        Eigen::MatrixXd intermediate_intersection;
        assert(np == sparse_basis_Z_s.rows());
        Eigen::MatrixXd new_sbnp_basis;
        new_sbnp_basis.setZero();
        tol = 1e-7;
        int ret;
        ret = getKernelSpaceBasis(sparse_basis_Z_s.transpose(),new_sbnp_basis,tol,true);
        assert(ret == 0);
        //To avoid warnings
        ((void)ret);
        std::cout << "new_sbnp_basis.cols: " << new_sbnp_basis.cols() << std::endl;
        std::cout << "sparse_basis_Z_s cols " << sparse_basis_Z_s.cols() << std::endl;
        ret = getSubSpaceIntersection(new_sbnp_basis,B_nu_i,intermediate_intersection,tol);
        assert(ret == 0);
        std::cout << "test 1" << std::endl;
                std::cout << "~~~" << std::endl;

        std::cout << sparse_basis_Z_s.transpose()*intermediate_intersection << std::endl;
                std::cout << "~~~" << std::endl;

        std::cout << (Eigen::MatrixXd::Identity(np,np)-B_nu_i*B_nu_i.transpose())*intermediate_intersection << std::endl;
        std::cout << "~~~" << std::endl;


        ret = getSubSpaceIntersection(dense_basis,intermediate_intersection,new_sparse_basis,tol);
        assert(ret == 0);

        std::cout << "test 2" << std::endl;
        std::cout << "~~~" << std::endl;
        std::cout << (Eigen::MatrixXd::Identity(np,np)-intermediate_intersection*intermediate_intersection.transpose())*new_sparse_basis << std::endl;
        std::cout << "~~~" << std::endl;
        std::cout << (Eigen::MatrixXd::Identity(np,np)-dense_basis*dense_basis.transpose())*new_sparse_basis << std::endl;
        std::cout << "~~~" << std::endl;


        //std::cout << "intermediate_intersection size" << intermediate_intersection.rows() << " " << intermediate_intersection.cols() << std::endl;
        std::cout << "~~~new_sparse_basis size~~~ " << new_sparse_basis.rows() << " " << new_sparse_basis.cols() << std::endl;
        sparse_basis_Z_s.conservativeResize(sparse_basis_Z_s.rows(),sparse_basis_Z_s.cols()+new_sparse_basis.cols());
        sparse_basis_Z_s.block(0,sparse_basis_Z_s.cols()-new_sparse_basis.cols(),np,new_sparse_basis.cols()) = new_sparse_basis;
    }

    std::cout << "Sparse basis cols: " << sparse_basis_Z_s.cols() << " other " << dense_basis.cols() << std::endl;
    assert(sparse_basis_Z_s.cols() == dense_basis.cols());

    basis = sparse_basis_Z_s;

    //Reverting order of columns for a more reasonable result
    for(int col=0; col < basis.cols(); col++ ) {
        basis.block(0,col,np,1) = sparse_basis_Z_s.block(0,sparse_basis_Z_s.cols()-1-col,np,1);
    }

    return 0;
}


std::string DynamicRegressorGenerator::analyseBaseSubspace(const Eigen::MatrixXd & basis, int verbosity_level, double tol)
{
    if(basis.rows() != getNrOfParameters()) { return "DynamicRegressorGenerator::analyseBaseSubspace error in input basis\n"; }

    if( tol <= 0.0 ) {
        tol = 1e-5;
    }

    std::stringstream ss;

    int np = getNrOfParameters();
    int np_completly_not_identifiable = 0;
    int np_completly_identifiable = 0;
    int np_partially_identifiable = 0;

    Eigen::MatrixXd full_basis_projection = basis.transpose();

    for(int p=0; p < getNrOfParameters(); p++ ) {
        enum{ COMPLETLY_IDENT, PARTIALLY_IDENT, COMPLETLY_NOT_IDENT } identifiability;

        assert(p < full_basis_projection.cols());
        double project_norm = full_basis_projection.block(0,p,full_basis_projection.rows(),1).norm();

        if( fabs(project_norm-1) < tol ) {
            identifiability = COMPLETLY_IDENT;
            np_completly_identifiable++;
        } else if ( fabs(project_norm) < tol ) {
            identifiability = COMPLETLY_NOT_IDENT;
            np_completly_not_identifiable++;
        } else {
            identifiability = PARTIALLY_IDENT;
            np_partially_identifiable++;
        }


        if( identifiability == COMPLETLY_IDENT )
        {
            ss << "Par. n. " << p << "(" << getDescriptionOfParameter(p) << ") is:\t\t";
            ss << "COMP.     IDENTIFIABLE" << " (pj norm: " << project_norm << ")" << std::endl;
        }
        else if ( identifiability == COMPLETLY_NOT_IDENT && verbosity_level > 1 )
        {
            ss << "Par. n. " << p << "(" << getDescriptionOfParameter(p) << ") is:\t\t";
            ss << "COMP. NOT IDENTIFIABLE" << " (pj norm: " << project_norm << ")" << std::endl;
        }
        else if( identifiability == PARTIALLY_IDENT )
        {
            ss << "Par. n. " << p << "(" << getDescriptionOfParameter(p) << ") is:\t\t";
            ss << "PARTIALLY IDENTIFIABLE" << " (pj norm: " << project_norm << ")" << std::endl;
        }
    }

    ss << "Num of total parameters:        : " << np << std::endl;
    ss << "Num of base parameters:         : " << basis.cols() << std::endl;
    ss << "Num of comp. identifiable params: " << np_completly_identifiable << std::endl;
    ss << "Num of comp. not identifiable   : " << np_completly_not_identifiable << std::endl;
    ss << "Num of partially identifiable   : " << np_partially_identifiable << std::endl;

    return ss.str();
}

std::string DynamicRegressorGenerator::analyseSparseBaseSubspace(const Eigen::MatrixXd & basis, double tol, bool only_summary)
{
    if(basis.rows() != getNrOfParameters()) { return "DynamicRegressorGenerator::analyseBaseSubspace error in input basis\n"; }

    if( tol <= 0.0 ) {
        tol = 1e-5;
    }

    std::stringstream ss;

    //int np = getNrOfParameters();
    int nbp = basis.cols();
    int nbp_I = 0;
    int nbp_II = 0;
    int nbp_III = 0;

    Eigen::MatrixXd full_basis_projection = basis.transpose();

    int relative_link = -1;
    int parameter_type = -1; //Can be 1,2 or 3
    int comp_ident_parameter = -1;

    for(int bp=0; bp < (int)nbp; bp++ ) {
        //For a given base parameter, we try to understand is type
        relative_link = -1;
        parameter_type = -1;
        for(int link_id = 0; link_id < (int)undirected_tree.getNrOfLinks(); link_id++) {
            //std::cout << "analyseSparseBaseSubspace: bp " << bp << " considering link " << link_id << std::endl;
            if( linkIndeces2regrColumns[link_id] == -1 ) { continue; } // fake link, don't consider
            int regr_link_id = linkIndeces2regrColumns[link_id];

            for( int local_param=0; local_param < 10; local_param++ ) {
                int p = 10*regr_link_id+local_param;
                /*
                std::cout << "analyseSparseBaseSubspace: p" << p << std::endl;
                std::cout << "relative_link " << relative_link << std::endl;
                std::cout << "parameter_type " << parameter_type << std::endl;
                std::cout << "value element " << fabs(full_basis_projection(bp,p)) << std::endl;
                */
                if( fabs(full_basis_projection(bp,p)) > tol ) {
                    if( relative_link == -1 ) {
                        //Check the first nonzero parameter
                        relative_link = regrColumns2linkIndeces[regr_link_id];
                        //if the element of the matrix is 1, the base parameter is of type III
                        if(fabs(full_basis_projection(bp,p)-1) < tol ) {
                            parameter_type = 3;
                            comp_ident_parameter = p;
                        } else {
                            //otherwise, if the vector is all in space of
                            parameter_type = 2;
                            comp_ident_parameter = -1;
                        }
                    } else {
                        if( regrColumns2linkIndeces[regr_link_id] != relative_link ) {
                            parameter_type = 1;
                            comp_ident_parameter = -1;
                            //and then we can exit from the loop
                        }
                    }
                }


            }
        }


       if( !only_summary) ss << "Base parameter " << bp << " is of type ";
       switch(parameter_type) {
                case(1):
                    nbp_I++;
                    if( !only_summary ) ss << "I: is a linear combination of the inertial parameters of the subtree starting at link " << undirected_tree.getLink(relative_link)->getName() << " ( " << relative_link << " ) "  << std::endl;
                break;
                case(2):
                    nbp_II++;
                    if( !only_summary ) ss << "II: is a linear combination of the inertial parameters of link " << undirected_tree.getLink(relative_link)->getName() << " ( " << relative_link << " ) "  << std::endl;
                break;
                case(3):
                    nbp_III++;
                    if( !only_summary ) ss << "III: is the parameter " << getDescriptionOfParameter(comp_ident_parameter) << std::endl;
                break;
                default:
                    assert(false);
                break;
            }


    }

    ss << "Num of type I   base parameters: " << nbp_I << std::endl;
    ss << "Num of type II  base parameters: " << nbp_II << std::endl;
    ss << "Num of type III base parameters: " << nbp_III << std::endl;

    return ss.str();
}



int DynamicRegressorGenerator::setFTSensorMeasurement(const int ft_sensor_index, const iDynTree::Wrench ftm)
{
    if( ft_sensor_index < 0 || ft_sensor_index >= sensorsList.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) )
    {
        if( verbose )
        {
            std::cerr << "DynamicsRegressorGenerator::setFTSensorMeasurement error: ft_sensor_index "
                      << ft_sensor_index  << " out of bounds" << std::endl;
        }

        return -1;
    }

    sensorMeasures.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft_sensor_index,ftm);

    return 0;
}

int DynamicRegressorGenerator::setTorqueSensorMeasurement(const int dof_index, const double measure)
{
    if( dof_index < 0 || dof_index >= NrOfDOFs ) {
        if( verbose ) std::cerr << "DynamicsRegressorGenerator::setTorqueSensorMeasurement error: dof_index " << dof_index  << " out of bounds" << std::endl;
        return -1;
    }

    measured_torques(dof_index) = measure;

    return 0;
}

int DynamicRegressorGenerator::setTorqueSensorMeasurement(const KDL::JntArray & torques)
{
    if( torques.rows() != measured_torques.rows() ) {
        if( verbose ) std::cerr << "DynamicsRegressorGenerator::setTorqueSensorMeasurement error" << std::endl;
        return -1;
    }

    measured_torques = torques;

    return 0;
}


int DynamicRegressorGenerator::addSubtreeRegressorRows(const std::vector< std::string>& _subtree_leaf_links)
{
    DynamicRegressorInterface * new_regr;
    subtreeBaseDynamicsRegressor * new_st_regr;

    new_st_regr = new subtreeBaseDynamicsRegressor(undirected_tree,sensorsList,linkIndeces2regrColumns,_subtree_leaf_links,consider_ft_offset,verbose);

    new_regr = (DynamicRegressorInterface *) new_st_regr;

    int ret_val = new_st_regr->configure();
    if( ret_val != 0 ) {
        if( verbose ) { std::cerr << "DynamicRegressorGenerator::addSubtreeRegressorRows error: could not load regressor with error " << ret_val << std::endl; }
        delete new_st_regr;
        return -1;
    }

    subtree_regressors.push_back(new_st_regr);
    regressors_ptrs.push_back(new_regr);

    NrOfOutputs += new_regr->getNrOfOutputs();

    parameters_desc.addList(new_regr->getUsedParameters());
    updateBuffers();
    configure();

    return 0;
}


int DynamicRegressorGenerator::addTorqueRegressorRows(const std::string & dof_name, const bool reverse_direction, const std::vector<bool> &_activated_ft_sensors)
{
    DynamicRegressorInterface * new_regr;
    torqueRegressor * new_st_regr;

    new_st_regr = new torqueRegressor(undirected_tree,sensorsList,linkIndeces2regrColumns,dof_name,reverse_direction,_activated_ft_sensors,consider_ft_offset,verbose);

    new_regr = (DynamicRegressorInterface *) new_st_regr;

    int ret_val = new_st_regr->configure();
    if( ret_val != 0 ) {
        if( verbose ) { std::cerr << "DynamicRegressorGenerator::addTorqueRegressorRows error: could not load regressor with error " << ret_val << std::endl; }
        delete new_st_regr;
        return -1;
    }

    torque_regressors.push_back(new_st_regr);
    regressors_ptrs.push_back(new_regr);

    NrOfOutputs += new_regr->getNrOfOutputs();

    parameters_desc.addList(new_regr->getUsedParameters());
    updateBuffers();
    configure();

    return 0;
}


int DynamicRegressorGenerator::addTorqueRegressorRows(const std::string & dof_name, const bool reverse_direction, const std::vector<std::string> &_activated_ft_sensors)
{
    unsigned int NrOfFTSensors = sensorsList.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);
    std::vector<bool> flag_activated_ft_sensors(NrOfFTSensors,false);
    for(int i=0; i < (int)_activated_ft_sensors.size(); i++ ) {
       KDL::CoDyCo::LinkMap::const_iterator link_it =  undirected_tree.getLink(_activated_ft_sensors[i]);
       if( link_it == undirected_tree.getInvalidLinkIterator() )
       {
            if( verbose )
            {
                std::cerr << "DynamicRegressorGenerator::addTorqueRegressorRows error: link "
                          << _activated_ft_sensors[i]  << " does not exists" << std::endl;
            }
            return -1;
       }

       int ft_id = getFirstFTSensorOnLink(sensorsList,link_it->getLinkIndex());
       {
           if( ft_id >= (int)flag_activated_ft_sensors.size() || ft_id < 0 )
           {
               if( verbose )
               {
                   std::cerr << "DynamicRegressorGenerator::addTorqueRegressorRows error while adding subtree with leaf: "
                             << _activated_ft_sensors[i]  << std::endl;
               }
               return -1;
           }
           flag_activated_ft_sensors[ft_id] = true;
       }
    }
    return addTorqueRegressorRows(dof_name,reverse_direction,flag_activated_ft_sensors);
}


int DynamicRegressorGenerator::addAllTorqueRegressorRows()
{
    int ret_val;
    for(int i=0; i < (int)undirected_tree.getNrOfDOFs(); i++ ) {
        ret_val = addTorqueRegressorRows(undirected_tree.getJunction(i)->getName());
        if( ret_val != 0 ) { return ret_val; }
    }
    return 0;
}

int DynamicRegressorGenerator::addBaseRegressorRows()
{
    DynamicRegressorInterface * new_regr;
    baseDynamicsRegressor * new_base_regr;

    new_base_regr = new baseDynamicsRegressor(undirected_tree,linkIndeces2regrColumns,verbose);

    new_regr = (DynamicRegressorInterface *) new_base_regr;

    int ret_val = new_base_regr->configure();
    if( ret_val != 0 ) {
        if( verbose ) { std::cerr << "DynamicRegressorGenerator::addBaseRegressorRows error: could not load regressor with error " << ret_val << std::endl; }
        delete new_base_regr;
        return -1;
    }

    base_regressors.push_back(new_base_regr);
    regressors_ptrs.push_back(new_regr);

    NrOfOutputs += new_regr->getNrOfOutputs();

    parameters_desc.addList(new_regr->getUsedParameters());
    updateBuffers();
    configure();

    return 0;
}

bool DynamicRegressorGenerator::configure()
{
    bool ok = true;
    for(unsigned int subRegr=0; subRegr < regressors_ptrs.size(); subRegr++ )
    {
        ok = ok && regressors_ptrs[subRegr]->setGlobalParameters(this->parameters_desc);
    }

    return ok;
}

int DynamicRegressorGenerator::updateBuffers()
{
    one_rows_buffer = Eigen::MatrixXd(1,parameters_desc.getNrOfParameters());
    six_rows_buffer = Eigen::MatrixXd(6,parameters_desc.getNrOfParameters());

    one_rows_vector = Eigen::VectorXd(1);
    six_rows_vector = Eigen::VectorXd(6);

    return -1;
}

int DynamicRegressorGenerator::getUpdatedModel(const Eigen::VectorXd & values,
                                               KDL::CoDyCo::UndirectedTree & updated_model)
{
    if( values.rows() != parameters_desc.getNrOfParameters() )
    {
        return -1;
    }

    updated_model = undirected_tree;

    /** \todo TODO remove dynamic memory allocation */
    Eigen::VectorXd inertial_parameters = values.segment(0,10*NrOfRealLinks_gen);

    assert(fake_links_names.size()+(inertial_parameters.rows()/10)==undirected_tree.getNrOfLinks());

    inertialParametersVectorToUndirectedTreeLoopFakeLinks(inertial_parameters,updated_model,fake_links_names);

    return 0;
}

int DynamicRegressorGenerator::getModelParameters(Eigen::VectorXd & values)
{
    if( values.rows() != parameters_desc.getNrOfParameters() )
    {
        values.resize(parameters_desc.getNrOfParameters());
    }

    /** \todo TODO remove dynamic memory allocation */
    values.setZero();

    // Fill only inertial parameters, no offset information is provided in the model
    for(unsigned int paramIndex=0; paramIndex < parameters_desc.parameters.size(); paramIndex++ )
    {
        iDynTree::Regressors::DynamicsRegressorParameter param = parameters_desc.parameters[paramIndex];
        if( param.category == iDynTree::Regressors::LINK_PARAM )
        {
            Eigen::Matrix<double,10,1> linkInertialParameters = KDL::CoDyCo::Vectorize(undirected_tree.getLink(param.elemIndex)->getInertia());
            values[paramIndex] = linkInertialParameters(getInertialParameterLocalIndex(param.type));
        }
    }

    return 0;
}


}

}


}
