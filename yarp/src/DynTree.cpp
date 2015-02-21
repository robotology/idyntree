/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 */

#include "DynTree.h"
#include "yarp_kdl.h"

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

#include "kdl_codyco/six_axis_ft_sensor.hpp"

#include <kdl_format_io/urdf_import.hpp>


#include <kdl/frames_io.hpp>

#ifndef NDEBUG
#include <kdl/frames_io.hpp>
#endif

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

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


DynTree::DynTree()
{
}

DynTree::DynTree(const KDL::Tree & _tree,
                   const std::vector<std::string> & joint_sensor_names,
                   const std::string & imu_link_name,
                   KDL::CoDyCo::TreeSerialization serialization
                   ):  undirected_tree(_tree,serialization)
{
    constructor(_tree,joint_sensor_names,imu_link_name,serialization);
}

DynTree::DynTree(const std::string urdf_file,
                 const std::vector<std::string> & joint_sensor_names,
                 const std::string & imu_link_name,
                 KDL::CoDyCo::TreeSerialization  serialization)
{
    KDL::Tree my_tree;
    if (!kdl_format_io::treeFromUrdfFile(urdf_file,my_tree))
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
    NrOfDynamicSubGraphs = NrOfFTSensors + 1;

    assert(undirected_tree.getNrOfDOFs() == NrOfDOFs);
    //Remve assertion for robot without a proper fixed base
    //assert((int)undirected_tree.getNrOfLinks() == NrOfLinks);

    world_base_frame = KDL::Frame::Identity();

    q = KDL::JntArray(NrOfDOFs);

    is_X_dynamic_base_updated = false;

    dq = KDL::JntArray(NrOfDOFs);
    ddq = KDL::JntArray(NrOfDOFs);

    torques = KDL::JntArray(NrOfDOFs);

    q_jnt_max = KDL::JntArray(NrOfDOFs);
    q_jnt_min = KDL::JntArray(NrOfDOFs);
    tau_max = KDL::JntArray(NrOfDOFs);

    constrained = std::vector<bool>(NrOfDOFs,false);
    constrained_count = 0;

    kinematic_traversal = KDL::CoDyCo::Traversal();
    dynamic_traversal = KDL::CoDyCo::Traversal();

    //measured_wrenches =  std::vector< KDL::Wrench >(NrOfFTSensors);
    sensor_measures.setNrOfSensors(KDL::CoDyCo::SIX_AXIS_FORCE_TORQUE,NrOfFTSensors);

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

    //iDynTreeContact
    ret = buildSubGraphStructure(joint_sensor_names);
    if( ret != 0 ) { std::cerr << "iDynTree constructor: ft sensor specified not found" << std::endl; }

    //building matrix and vectors for each subgraph
    contacts.resize(NrOfDynamicSubGraphs);
    A_contacts.resize(NrOfDynamicSubGraphs);
    b_contacts.resize(NrOfDynamicSubGraphs,Vector(6,0.0));
    x_contacts.resize(NrOfDynamicSubGraphs);

    b_contacts_subtree.resize(NrOfLinks);

    //end iDynTreeContact


    assert(ret == 0);
    correctly_configure = true;
}

DynTree::~DynTree() { }

double DynTree::setAng(const double q_in, const int i)
{
    is_X_dynamic_base_updated = false;

    if (constrained[i]) {
        q(i) = (q_in<q_jnt_min(i)) ? q_jnt_min(i) : ((q_in>q_jnt_max(i)) ? q_jnt_max(i) : q_in);
    } else {
        q(i) = q_in;
    }
    return q(i);
}


//====================================
//
//      iDynTreeContact methods
//
//====================================

bool DynTree::isFTsensor(const std::string & joint_name, const std::vector<std::string> & ft_sensors) const
{
    if (std::find(ft_sensors.begin(), ft_sensors.end(), joint_name) != ft_sensors.end())
    {
        return true;
    }
    //else
    {
        return false;
    }
}

bool DynTree::generateSensorsTree(const std::vector<std::string> & ft_names,
                                    const std::vector<bool> & is_measure_direction_child_to_parent)
{
    for(int i=0; i < ft_names.size(); i++ )
    {
        //Creating a new ft sensor to be added in the ft sensors structure
        KDL::CoDyCo::SixAxisForceTorqueSensor new_sens;


        if( this->undirected_tree.getJunction(ft_names[i]) != this->undirected_tree.getInvalidJunctionIterator() )
        {
            //Set the sensor name (for the time being equal to the junction name)
            new_sens.setName(ft_names[i]);
            //Set the junction name
            new_sens.setParent(ft_names[i]);
            int junction_index = this->undirected_tree.getJunction(ft_names[i])->getJunctionIndex();
            new_sens.setParentIndex(junction_index);
            KDL::CoDyCo::JunctionMap::const_iterator junct_it = this->undirected_tree.getJunction(ft_names[i]);

            int parent_index = junct_it->getParentLink()->getLinkIndex();
            int child_index = junct_it->getChildLink()->getLinkIndex();
            std::string parent_link_name = junct_it->getParentLink()->getName();
            std::string child_link_name = junct_it->getChildLink()->getName();

            if( is_measure_direction_child_to_parent[i] )
            {
                new_sens.setAppliedWrenchLink(parent_index);
            }
            else
            {
                new_sens.setAppliedWrenchLink(child_index);
            }

            // Currently we support only the case where the ft sensor frame is equal
            // to the child link frame
            new_sens.setSecondLinkSensorTransform(child_index,KDL::Frame::Identity());

            // Then, the parent_link_H_sensor transform is simply parent_link_H_child_link transform
            KDL::Frame parent_link_H_sensor = junct_it->pose(0.0,false);
            new_sens.setFirstLinkSensorTransform(parent_index,parent_link_H_sensor);

        }
        else
        {
            std::cerr << "[ERR] DynTree::generateSensorsTree: problem generating sensor for ft "
                      << ft_names[i] << std::endl;
            return false;
        }

        int ret = sensors_tree.addSensor(KDL::CoDyCo::SIX_AXIS_FORCE_TORQUE,new_sens);

        assert(ret == i);
    }

    return true;
}


int DynTree::buildSubGraphStructure(const std::vector<std::string> & ft_names)
{
    link2subgraph_index.resize(NrOfLinks,-1);
    link_is_subgraph_root.resize(NrOfLinks,false);
    subgraph_index2root_link.resize(NrOfDynamicSubGraphs,-1);

    int next_id = 0;

    for(int i=0; i < dynamic_traversal.getNrOfVisitedLinks(); i++) {

        KDL::CoDyCo::LinkMap::const_iterator link_it = dynamic_traversal.getOrderedLink(i);
        int link_nmbr = link_it->getLinkIndex();

        if( i == 0 ) {

            //Starting with the dynamical base link, assign an index to the subgraph
            assert( dynamic_traversal.getParentLink(link_nmbr) == undirected_tree.getInvalidLinkIterator() );
            link2subgraph_index[link_nmbr] = next_id;

            //The dynamical base link is the root of its subgraph
            link_is_subgraph_root[link_nmbr] = true;
            subgraph_index2root_link[link2subgraph_index[link_nmbr]] = link_nmbr;

            next_id++;

        } else {
            //For every link, the subgraph is the same of its parent, unless it is connected to it by an FT sensor
            KDL::CoDyCo::LinkMap::const_iterator parent_it = dynamic_traversal.getParentLink(link_it->getLinkIndex());
            int parent_nmbr = parent_it->getLinkIndex();

            if( isFTsensor(link_it->getAdjacentJoint(parent_it)->getJoint().getName(),ft_names) ) {
                //The FT sensor should be a fixed joint ? probably not
                //assert(link_it->getAdjacentJoint(parent_it)->joint.getType() == Joint::None);

                link2subgraph_index[link_nmbr] = next_id;

                //This link is a root of a dynamical subgraph, as its parent is in another subgraph
                link_is_subgraph_root[link_nmbr] = true;
                subgraph_index2root_link[link2subgraph_index[link_nmbr]] = link_nmbr;

                next_id++;
            } else {
                link2subgraph_index[link_nmbr] = link2subgraph_index[parent_nmbr];

                //This link is not a root of a dynamical subgraph
                link_is_subgraph_root[link_nmbr] = false;
            }
        }
    }

    //Building Force/Torque sensors data structures
    std::vector<bool> is_measure_direction_child_to_parent(ft_names.size(),true);
    //ft_list = KDL::CoDyCo::FTSensorList(undirected_tree,ft_names,is_measure_direction_child_to_parent);
    this->generateSensorsTree(ft_names,is_measure_direction_child_to_parent);

    //The numbers of ids must be equal to the number of subgraphs
    if(next_id == NrOfDynamicSubGraphs) {
        return 0;
    } else {
        assert(false);
        return -1;
    }
}

KDL::Wrench DynTree::getMeasuredWrench(int link_id)
{

    KDL::Wrench total_measured_applied_wrench = KDL::Wrench::Zero();
    for(int ft=0; ft < NrOfFTSensors; ft++ )
    {
        KDL::CoDyCo::SixAxisForceTorqueSensor * sens
            = (KDL::CoDyCo::SixAxisForceTorqueSensor *) sensors_tree.getSensor(KDL::CoDyCo::SIX_AXIS_FORCE_TORQUE,ft);

        assert(sens != 0);

        KDL::Wrench measured_wrench_on_link = KDL::Wrench::Zero();
        KDL::Wrench measured_wrench_by_sensor;

        bool ok = sensor_measures.getMeasurement(KDL::CoDyCo::SIX_AXIS_FORCE_TORQUE,ft,measured_wrench_by_sensor);

        assert(ok);

        // If the sensor with index ft is not attached to the link
        // this function return a zero wrench
        sens->getWrenchAppliedOnLink(link_id,measured_wrench_by_sensor,measured_wrench_on_link);

        //Sum the given wrench to the return value
        total_measured_applied_wrench += measured_wrench_on_link;
    }

    return total_measured_applied_wrench;
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

void DynTree::buildAb_contacts()
{
//    #ifndef NDEBUG
//    bool extreme_verbose = false;
//    #endif
    //First calculate the known terms b related to inertial, gravitational and
    //measured F/T

    for(int l=dynamic_traversal.getNrOfVisitedLinks()-1; l>=0; l-- ) {
            KDL::CoDyCo::LinkMap::const_iterator link_it = dynamic_traversal.getOrderedLink(l);
            int link_nmbr = link_it->getLinkIndex();
            //Collect RigidBodyInertia and external forces
            KDL::RigidBodyInertia Ii= link_it->getInertia();
            //This calculation should be done one time in forward kineamtic loop and stored \todo
            b_contacts_subtree[link_nmbr] = Ii*a[link_nmbr]+v[link_nmbr]*(Ii*v[link_nmbr]) - getMeasuredWrench(link_nmbr);
            #ifndef NDEBUG
            /*
            if(extreme_verbose) {
            std::cerr << "link_nmbr : " << link_nmbr << std::endl;
            std::cerr << "b_contacts_subtree: " << b_contacts_subtree[link_nmbr] << std::endl;
            std::cerr << "a " << a[link_nmbr] << std::endl;
            std::cerr << "v " << v[link_nmbr] << std::endl;
            std::cerr << "getMeasuredWrench " << getMeasuredWrench(link_nmbr) << std::endl;
            }
            */
            #endif
    }


    for(int l=dynamic_traversal.getNrOfVisitedLinks()-1; l>=0; l-- ) {

        KDL::CoDyCo::LinkMap::const_iterator link_it = dynamic_traversal.getOrderedLink(l);
        int link_nmbr = link_it->getLinkIndex();

        if( l != 0 ) {

            KDL::CoDyCo::LinkMap::const_iterator parent_it = dynamic_traversal.getParentLink(link_nmbr);
            const int parent_nmbr = parent_it->getLinkIndex();
            //If this link is a subgraph root, store the result, otherwise project it to the parent
            #ifndef NDEBUG
            //std::cerr << "Link_nmbr " << link_nmbr << std::endl;
            //std::cerr << "isSubGraphRoot(" << link_nmbr << ") " << isSubGraphRoot(link_nmbr) << std::endl;
            #endif
            if( !isSubGraphRoot(link_nmbr) ) {
                double joint_pos;

                KDL::CoDyCo::JunctionMap::const_iterator joint_it = link_it->getAdjacentJoint(parent_it);
                if( joint_it->getJoint().getType() == KDL::Joint::None ) {
                    joint_pos = 0.0;
                } else {
                    joint_pos = q(link_it->getAdjacentJoint(parent_it)->getDOFIndex());
                }
                b_contacts_subtree[parent_nmbr] += link_it->pose(parent_it,joint_pos)*b_contacts_subtree[link_nmbr];
            }
       }

       if( isSubGraphRoot(link_nmbr ) )
       {
           //std::cout << " setting b_contact [ " << getSubGraphIndex(link_nmbr) << " ] ";
           //std::cout << "to " << b_contacts_subtree[link_nmbr] << std::endl;
           b_contacts[getSubGraphIndex(link_nmbr)].setSubvector(0,KDLtoYarp(b_contacts_subtree[link_nmbr].torque));
           b_contacts[getSubGraphIndex(link_nmbr)].setSubvector(3,KDLtoYarp(b_contacts_subtree[link_nmbr].force));
           //std::cout << " b_contacts [ " << getSubGraphIndex(link_nmbr) << " ] is " << b_contacts[getSubGraphIndex(link_nmbr)].toString() << std::endl;
       }


    }

    //Then calculate the A and b related to unknown contacts

    iCub::skinDynLib::dynContactList::const_iterator it;

    std::vector<int> unknowns(NrOfDynamicSubGraphs,0);

    //Calculate the number of unknowns for each subgraph
    for(int sg=0; sg < NrOfDynamicSubGraphs; sg++ ) {
        for(it = contacts[sg].begin(); it!=contacts[sg].end(); it++)
        {
            //get link index
            int skinDynLib_body_part = it->getBodyPart();
            int skinDynLib_link_index = it->getLinkNumber();

            int iDynTree_link_index = -1;
            int iDynTree_skinFrame_index = -1;

            bool skinDynLib_ID_found = skinDynLib2iDynTree(skinDynLib_body_part,skinDynLib_link_index,
                                                           iDynTree_link_index,iDynTree_skinFrame_index);

            if( !skinDynLib_ID_found )
            {
                std::cerr << "[ERR] DynTree::buildAb_contacts() not found, skipping contact" << std::endl;
                continue;
            }

            if(it->isMomentKnown())
            {
                if(it->isForceDirectionKnown())
                {
                    unknowns[sg]++;     // 1 unknown (force module)
                }
                else
                {
                    unknowns[sg]+=3;    // 3 unknowns (force)
                }
            }
            else
            {
                unknowns[sg]+=6;        // 6 unknowns (force and moment)
            }
        }
    }


    for(int sg=0; sg < NrOfDynamicSubGraphs; sg++ ) {
        //Resize the A matrices
        A_contacts[sg] = yarp::sig::Matrix(6,unknowns[sg]);

        //Calculate A and b related to contacts
        int colInd = 0;
        for( it = contacts[sg].begin(); it!=contacts[sg].end(); it++)
        {
            //get link index
            int skinDynLib_body_part = it->getBodyPart();
            int skinDynLib_link_index = it->getLinkNumber();

            int iDynTree_link_index = -1;
            int iDynTree_skinFrame_index = -1;

            bool skinDynLib_ID_found = skinDynLib2iDynTree(skinDynLib_body_part,skinDynLib_link_index,
                                                           iDynTree_link_index,iDynTree_skinFrame_index);

            if( !skinDynLib_ID_found )
            {
                std::cerr << "[ERR] DynTree::buildAb_contacts() not found, skipping contact" << std::endl;
                continue;
            }

            //Subgraph information
            int subgraph_index = link2subgraph_index[iDynTree_link_index];

            assert(subgraph_index == sg);

            // We will write the force estimation minimization problem in
            // the plucker frame of the root link of the subgraph
            int subgraph_root = subgraph_index2root_link[subgraph_index];

            //Get Frame transform between contact link and subgraph root
            //Inefficient but leads to cleaner code, if necessary can be improved
            KDL::Frame H_root_link = getFrameLoop(undirected_tree,q,dynamic_traversal,subgraph_root,iDynTree_link_index);
            // \todo TODO this transformation is constant, just store it somewhere
            KDL::Frame H_link_skinFrame = getFrameLoop(undirected_tree,q,dynamic_traversal,iDynTree_link_index,iDynTree_skinFrame_index);

            // skinDynLib express the force in a frame that can be
            // different from iDynTree link frame, we will call this
            // skinFrame . The COP got from the the skinContactList is
            // expressed in this skinFrame, and we will estimate the force
            // and the torque in this skinFrame because the force will be
            // still streamed out in a skinContactList form.
            KDL::Vector COP_skinFrame;
            YarptoKDL(it->getCoP(),COP_skinFrame);

            // An additional frame is the frame oriented as the skinFrame, but
            // with the origin in the contact COP . We will call this frame the
            // "contactFrame"

            KDL::Frame H_skinFrame_contactFrame = KDL::Frame(COP_skinFrame);
            KDL::Frame H_root_contactFrame = H_root_link*H_link_skinFrame*H_skinFrame_contactFrame;

            if(it->isForceDirectionKnown())
            {
                // 1 UNKNOWN: FORCE MODULE
                yarp::sig::Matrix un(6,1);
                un.zero();
                un.setSubcol(it->getForceDirection(),3,0); // force direction unit vector
                yarp::sig::Matrix H_adj_root_contact = KDLtoYarp_wrench(H_root_contactFrame);
                yarp::sig::Matrix col = H_adj_root_contact*un;
                A_contacts[sg].setSubmatrix(col,0,colInd);
                colInd += 1;

            }
            else
            {
                if( it->isMomentKnown() ) {
                    // 3 UNKNOWNS: FORCE
                    A_contacts[sg].setSubmatrix(KDLtoYarp_wrench(H_root_contactFrame).submatrix(0,5,3,5),0,colInd);
                    colInd += 3;

                } else {
                    // 6 UNKNOWNS: FORCE AND MOMENT
                    A_contacts[sg].setSubmatrix(KDLtoYarp_wrench(H_root_contactFrame),0,colInd);
                    colInd += 6;
                }

            }
        }
    }


}

void DynTree::store_contacts_results()
{
    //Make sure that the external forces are equal to zero before storing the results
    for(int l=dynamic_traversal.getNrOfVisitedLinks()-1; l>=0; l-- ) {
        f_ext[dynamic_traversal.getOrderedLink(l)->getLinkIndex()] = KDL::Wrench::Zero();
    }

    for(int sg=0; sg < NrOfDynamicSubGraphs; sg++ ) {
        unsigned int unknownInd = 0;
        iCub::skinDynLib::dynContactList::iterator it;
        for(it = contacts[sg].begin(); it!=contacts[sg].end(); it++)
        {
            //get link index
            int skinDynLib_body_part = it->getBodyPart();
            int skinDynLib_link_index = it->getLinkNumber();

            int iDynTree_link_index = -1;
            int iDynTree_skinFrame_index = -1;

            bool skinDynLib_ID_found = skinDynLib2iDynTree(skinDynLib_body_part,skinDynLib_link_index,
                                                           iDynTree_link_index,iDynTree_skinFrame_index);

            if( !skinDynLib_ID_found )
            {
                std::cerr << "[ERR] DynTree::buildAb_contacts() not found, skipping contact" << std::endl;
                continue;
            }

            //Store the result in dynContactList, for output
            if(it->isForceDirectionKnown()) {
                //1 UNKNOWN
                it->setForceModule( x_contacts[sg](unknownInd++));
            }
            else
            {
                if(it->isMomentKnown())
                {
                    //3 UNKNOWN
                    assert( unknownInd+2 < x_contacts[sg].size() );
                    it->setForce(x_contacts[sg].subVector(unknownInd, unknownInd+2));
                    unknownInd += 3;
                } else {
                    //6 UNKNOWN
                    assert( unknownInd+2 < x_contacts[sg].size() );
                    it->setMoment(x_contacts[sg].subVector(unknownInd, unknownInd+2));
                    unknownInd += 3;
                    assert( unknownInd+2 < x_contacts[sg].size() );
                    it->setForce(x_contacts[sg].subVector(unknownInd, unknownInd+2));
                    unknownInd += 3;

                }
            }

            //Store the results in f_ext, for RNEA dynamic loop
            // f_ext is expressed in the link Plucker frame,
            // while skinContactList wrenches are expressed in the contactFrame
            KDL::Vector COP, force, moment;
            YarptoKDL(it->getCoP(),COP);
            KDL::Frame H_skinFrame_contactFrame = KDL::Frame(COP);
            YarptoKDL(it->getForce(),force);
            YarptoKDL(it->getMoment(),moment);

            KDL::Wrench f_ext_contactFrame = KDL::Wrench(force,moment);

            // Todo store and avoid repeated computations
            KDL::Frame H_link_skinFrame = getFrameLoop(undirected_tree,q,dynamic_traversal,iDynTree_link_index,iDynTree_skinFrame_index);

            KDL::Frame H_link_contactFrame = H_link_skinFrame*H_skinFrame_contactFrame;

            f_ext[iDynTree_link_index] = H_link_contactFrame*f_ext_contactFrame;
        }
    }
}

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
    if( com_yarp.size() != 3 ) { com_yarp.resize(3); }
    com_yarp.zero();
    return setInertialMeasureAndLinearVelocity(com_yarp,w0,ddp0,dw0);
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

    KDL::Wrench measured_wrench;
    YarptoKDL(ftm.subVector(0,2),measured_wrench.force);
    YarptoKDL(ftm.subVector(3,5),measured_wrench.torque);

    bool ret = sensor_measures.setMeasurement(KDL::CoDyCo::SIX_AXIS_FORCE_TORQUE,sensor_index,measured_wrench);

    are_contact_estimated = false;

    return ret;
}

bool DynTree::getSensorMeasurement(const int sensor_index, yarp::sig::Vector &ftm) const
{
    //\todo avoid unnecessary memory allocation
    yarp::sig::Vector force_yarp(3);
    yarp::sig::Vector torque_yarp(3);
    if( sensor_index < 0 ||
        sensor_index > (int)NrOfFTSensors )
    {
        return false;
    }

    if( ftm.size() != 6 )
    {
        ftm.resize(6);
    }

    KDL::Wrench measured_wrench;

    bool ok = sensor_measures.getMeasurement(KDL::CoDyCo::SIX_AXIS_FORCE_TORQUE,sensor_index,measured_wrench);

    assert(ok);

    KDLtoYarp(measured_wrench.force,force_yarp);
    KDLtoYarp(measured_wrench.torque,torque_yarp);

    ftm.setSubvector(0,force_yarp);
    ftm.setSubvector(3,torque_yarp);
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

bool DynTree::setContacts(const iCub::skinDynLib::dynContactList & contacts_list)
{
    assert((int)contacts.size() == NrOfDynamicSubGraphs);
    for(int sg = 0; sg < NrOfDynamicSubGraphs; sg++ ) {
        contacts[sg].resize(0);
    }

    //Separate unknown contacts depending on their subgraph
    for(iCub::skinDynLib::dynContactList::const_iterator it = contacts_list.begin();
            it != contacts_list.end(); it++ )
    {
            //get link index
            int skinDynLib_body_part = it->getBodyPart();
            int skinDynLib_link_index = it->getLinkNumber();

            int iDynTree_link_index = -1;
            int iDynTree_skinFrame_index = -1;

            bool skinDynLib_ID_found = skinDynLib2iDynTree(skinDynLib_body_part,skinDynLib_link_index,
                                                           iDynTree_link_index,iDynTree_skinFrame_index);

            if( !skinDynLib_ID_found )
            {
                std::cerr << "[ERR] DynTree::buildAb_contacts() not found, skipping contact" << std::endl;
                continue;
            }


        int subgraph_id = getSubGraphIndex(iDynTree_link_index);

        contacts[subgraph_id].push_back(*it);
    }

    are_contact_estimated = false;

    return true;
}

const iCub::skinDynLib::dynContactList DynTree::getContacts() const
{
    iCub::skinDynLib::dynContactList all_contacts(0);


    for(int sg = 0; sg < NrOfDynamicSubGraphs; sg++ )
    {
        all_contacts.insert(all_contacts.end(),contacts[sg].begin(),contacts[sg].end());
    }

    return all_contacts;
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
            return true;
        }
        //else
        return false;
    } else {
        return true;
    }
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

bool DynTree::estimateContactForcesFromSkin()
{
    #ifndef NDEBUG
    //std::cout << "DynTree::estimateContactForces " << std::endl;
    #endif

    double tol = 1e-7; /**< value extracted from old iDynContact */
    buildAb_contacts();
    for(int i=0; i < NrOfDynamicSubGraphs; i++ ) {
        #ifndef NDEBUG
        /*
        std::cout << "A_contacts " << i << " has size " << A_contacts[i].rows() << " " << A_contacts[i].cols() << std::endl;
        std::cout << A_contacts[i].toString() << std::endl;
        std::cout << "b_contacts " << i << " has size " << b_contacts[i].size() << std::endl;
        std::cout << b_contacts[i].toString() << std::endl;
        */
        #endif
        x_contacts[i] = yarp::math::pinv(A_contacts[i],tol)*b_contacts[i];
        #ifndef NDEBUG

        /*
        std::string contacts_string = x_contacts[i].toString();

        std::cout << "x_contacts " << i << " has size " << x_contacts[i].size() << std::endl;
        std::cout << x_contacts[i].toString() << std::endl;
        */
        #endif
    }
    store_contacts_results();
    are_contact_estimated = true;
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
        }
        if(  base_residual_f.torque.Norm() > 1e-5 )
        {
            std::cout << "iDynTree WARNING: base_residual_f.torque.Norm() is " << base_residual_f.torque.Norm() << " instead of zero." << std::endl;
        }
        //Note: this (that no residual appears happens only for the proper selection of the provided dynContactList
    }
    else
    {
        //In case contacts forces where not estimated, the sensor values have
        //to be calculated from the RNEA
        for(int i=0; i < NrOfFTSensors; i++ )
        {
            //Todo add case that the force/wrench is the one of the parent ?
            KDL::Wrench measure_wrench;

            KDL::CoDyCo::SixAxisForceTorqueSensor * p_ft_sensor =
                reinterpret_cast<KDL::CoDyCo::SixAxisForceTorqueSensor *>(sensors_tree.getSensor(KDL::CoDyCo::SIX_AXIS_FORCE_TORQUE,i));

            bool ok = p_ft_sensor->simulateMeasurement(dynamic_traversal,f,measure_wrench);

            assert(ok);

            ok = sensor_measures.setMeasurement(KDL::CoDyCo::SIX_AXIS_FORCE_TORQUE,i,measure_wrench);

            assert(ok);
        }
    }
    return ret >= 0;
}

////////////////////////////////////////////////////////////////////////
////// COM related methods
////////////////////////////////////////////////////////////////////////


KDL::Vector DynTree::getCOMKDL(int link_index)
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

yarp::sig::Vector DynTree::getCOM(int link_index)
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
    yarp::sig::Matrix dummy;
    return getCOMJacobian(jac,dummy);
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

    computePositions();
    dynamicsRegressorLoop(undirected_tree,q,dynamic_traversal,X_dynamic_base,v,a,dynamics_regressor);

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

int DynTree::getLinkIndex(const std::string & link_name)
{
    KDL::CoDyCo::LinkMap::const_iterator link_it = undirected_tree.getLink(link_name);
    if( link_it == undirected_tree.getInvalidLinkIterator() ) { std::cerr << "DynTree::getLinkIndex : link " << link_name << " not found" << std::endl; return -1; }
    return link_it->getLinkIndex();
}


bool DynTree::getLinkName(const int link_index, std::string & link_name)
{
    if( link_index < 0 || link_index >= this->getNrOfLinks() )
    {
        return false;
    }
    link_name = undirected_tree.getLink(link_index)->getName();
    return true;
}

int DynTree::getFrameIndex(const std::string & frame_name)
{
    return getLinkIndex(frame_name);
}

bool DynTree::getFrameName(const int frame_index, std::string & frame_name)
{
    return getLinkName(frame_index, frame_name);
}


int DynTree::getDOFIndex(const std::string & dof_name)
{
    KDL::CoDyCo::JunctionMap::const_iterator junction_it = undirected_tree.getJunction(dof_name);
    if( junction_it == undirected_tree.getInvalidJunctionIterator() || junction_it->getNrOfDOFs() != 1 ) { std::cerr << "DynTree::getDOFIndex : DOF " << dof_name << " not found" << std::endl; return -1; }
    return junction_it->getDOFIndex();
}

bool DynTree::getDOFName(const int dof_index, std::string & dof_name)
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

int DynTree::getJunctionIndex(const std::string & junction_name)
{
    KDL::CoDyCo::JunctionMap::const_iterator junction_it = undirected_tree.getJunction(junction_name);
    if( junction_it == undirected_tree.getInvalidJunctionIterator() ) { std::cerr << "DynTree::getJunctionIndex : Junction " << junction_name << " not found" << std::endl; return -1; }
    return junction_it->getJunctionIndex();
}

bool DynTree::getJunctionName(const int junction_index, std::string & junction_name)
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
bool DynTree::getFTSensorName(const int /*junction_index*/, std::string & /*junction_name*/)
{
    return false;
}


int DynTree::getFTSensorIndex(const std::string & ft_name)
{
    KDL::CoDyCo::JunctionMap::const_iterator junction_it = undirected_tree.getJunction(ft_name);

    if( junction_it == undirected_tree.getInvalidJunctionIterator() )
    {
        std::cerr << "DynTree::getFTSensorIndex : junction " << ft_name << " not found" << std::endl;
        return -1;
    }

    if( junction_it->getNrOfDOFs() > 0 )
    {
        std::cerr << "DynTree::getFTSensorIndex warning: " << ft_name << " is not a fixed junction " << std::endl;
    }

    unsigned int junction_index = junction_it->getJunctionIndex();
    // Search the ft index given the associated junction index
    assert( sensors_tree.getNrOfSensors(KDL::CoDyCo::SIX_AXIS_FORCE_TORQUE) == NrOfFTSensors );
    for( int ft = 0; ft < NrOfFTSensors; ft++ )
    {
        KDL::CoDyCo::SixAxisForceTorqueSensor * p_ft_sensor =
                dynamic_cast<KDL::CoDyCo::SixAxisForceTorqueSensor *>(sensors_tree.getSensor(KDL::CoDyCo::SIX_AXIS_FORCE_TORQUE,ft));
        assert(p_ft_sensor != 0);
        int ft_junction_index = p_ft_sensor->getParentIndex();
        if( ft_junction_index == junction_index )
        {
            return ft;
        }
    }

    return -1;

    //return ft_list.getFTSensorID(junction_it->getJunctionIndex());
}

int DynTree::getIMUIndex(const std::string & imu_name)
{
    if( imu_name == kinematic_traversal.getBaseLink()->getName() ) {
        return 0;
    } else {
         std::cerr << "DynTree::getIMUIndex : IMU " << imu_name << " not found" << std::endl;
         return -1;
    }
}

// \todo TODO FIXME implement this method
bool DynTree::getIMUName(const int /*junction_index*/, std::string & /*junction_name*/)
{
    return false;
}

std::vector<yarp::sig::Vector> DynTree::getSubTreeInternalDynamics()
{
    computePositions();
    std::vector<yarp::sig::Vector> return_value(NrOfDynamicSubGraphs,Vector(6,0.0));

    std::vector<KDL::Wrench> return_value_kdl(NrOfDynamicSubGraphs,KDL::Wrench::Zero());

    for(int i=0; i < dynamic_traversal.getNrOfVisitedLinks(); i++ ) {
        KDL::CoDyCo::LinkMap::const_iterator link_it = dynamic_traversal.getOrderedLink(i);
        return_value_kdl[link2subgraph_index[link_it->getLinkIndex()]] += X_dynamic_base[link_it->getLinkIndex()]*f_gi[link_it->getLinkIndex()];
    }

    for(int i=0; i < NrOfDynamicSubGraphs; i++ ) {
        for(int j=0; j < 6; j++ ) {
            return_value[i][j] = return_value_kdl[i](j);
        }
    }

    return return_value;
}

bool DynTree::addSkinDynLibAlias(const std::string iDynTree_link_name, const std::string iDynTree_frame_name,
                                const int skinDynLib_body_part, const int skinDynLib_link_index)
{
   int iDynTree_link_index = this->getLinkIndex(iDynTree_link_name);
   if( iDynTree_link_index < 0 )
   {
       std::cerr << "[ERR] addSkinDynLibAlias : link " << iDynTree_link_name << " not found in the model " << std::endl;
   }

   int iDynTree_frame_index = this->getLinkIndex(iDynTree_frame_name);
   if( iDynTree_frame_index < 0 )
   {
       std::cerr << "[ERR] addSkinDynLibAlias : frame " << iDynTree_frame_name << " not found in the model " << std::endl;
   }

   skinDynLibLinkID sdl_id;
   sdl_id.body_part = skinDynLib_body_part;
   sdl_id.local_link_index = skinDynLib_link_index;

   iDynTreeLinkAndFrame idyntree_id;
   idyntree_id.link_index = iDynTree_link_index;
   idyntree_id.frame_index = iDynTree_frame_index;

   //Remove any existing alias for this link to avoid anomalies
   this->removeSkinDynLibAlias(iDynTree_link_name);

   skinDynLibLinkMap.insert(std::pair<skinDynLibLinkID,iDynTreeLinkAndFrame>(sdl_id,idyntree_id));

   return true;
}

bool DynTree::getSkinDynLibAlias(const std::string iDynTree_link_name, std::string & iDynTree_frame_name,
                                int & skinDynLib_body_part, int & skinDynLib_link_index)
{
   int iDynTree_link_index = this->getLinkIndex(iDynTree_link_name);
   if( iDynTree_link_index < 0 )
   {
       std::cerr << "[ERR] getSkinDynLibAlias : link " << iDynTree_link_name << " not found " << std::endl;
       return false;
   }

   skinDynLibLinkID sdl_id;
   sdl_id.body_part = skinDynLib_body_part;
   sdl_id.local_link_index = skinDynLib_link_index;

     // TODO \todo What is this crazyness??? Linear search of a map??
   for(std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::iterator it = skinDynLibLinkMap.begin();
       it != skinDynLibLinkMap.end(); it++ )
   {
       if( it->second.link_index == iDynTree_link_index )
       {
           skinDynLib_body_part = it->first.body_part;
           skinDynLib_link_index = it->first.local_link_index;
           int iDynTree_frame_index = it->second.frame_index;
           iDynTree_frame_name = undirected_tree.getLink(iDynTree_frame_index)->getName();
           break;
       }
   }

   return true;

}

//FIXME TODO \todo implemente this method with an appropriate data structure, such that
//  it has a complexity of O(1)
bool DynTree::getSkinDynLibAlias(const int iDynTree_link_index, int & iDynTree_frame_index,
                        int & skinDynLib_body_part, int & skinDynLib_link_index)
{
  if( iDynTree_link_index < 0 || iDynTree_link_index >= this->getNrOfLinks() ) return false;

  // TODO \todo What is this crazyness??? Linear search of a map??
   for(std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::iterator it = skinDynLibLinkMap.begin();
       it != skinDynLibLinkMap.end(); it++ )
   {
       if( it->second.link_index == iDynTree_link_index )
       {
           skinDynLib_body_part = it->first.body_part;
           skinDynLib_link_index = it->first.local_link_index;
           iDynTree_frame_index = it->second.frame_index;
           return true;
       }
   }

   return false;

}

bool DynTree::removeSkinDynLibAlias(std::string link)
{
   int link_index = this->getLinkIndex(link);
   if( link_index < 0 )
   {
       std::cerr << "[ERR] removeSkinDynLibAlias : link " << link << " not found " << std::endl;
   }

   for(std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::iterator it = skinDynLibLinkMap.begin();
       it != skinDynLibLinkMap.end(); it++ )
   {
       if( it->second.link_index == link_index )
       {
           skinDynLibLinkMap.erase(it);
           break;
       }
   }

   return true;
}

bool DynTree::skinDynLib2iDynTree(const int skinDynLib_body_part,
                                 const int skinDynLib_link_index,
                                 int & iDynTree_link_index,
                                 int & iDynTree_frame_index)
{
    skinDynLibLinkID skinID;
    skinID.body_part = skinDynLib_body_part;
    skinID.local_link_index = skinDynLib_link_index;

    std::map<skinDynLibLinkID,iDynTreeLinkAndFrame>::iterator it = skinDynLibLinkMap.find(skinID);

    if( it == skinDynLibLinkMap.end() )
    {
        std::cerr << "[ERR] DynTree::skinDynLib2iDynTree : skinDynLib link "
                  << skinDynLib_body_part << " " << skinDynLib_link_index << " not found " << std::endl;
        return false;
    }

    iDynTree_link_index  = it->second.link_index;
    iDynTree_frame_index = it->second.frame_index;

    return true;
}

bool DynTree::loadJointLimitsFromURDFFile(std::string urdfFile, KDL::CoDyCo::UndirectedTree undirectedTree, yarp::sig::Vector &yarpJointMinLimit, yarp::sig::Vector &yarpJointMaxLimit) {

        unsigned int NrOfDOFs = undirectedTree.getNrOfDOFs();
        KDL::JntArray kdlJointMinLimit(NrOfDOFs), kdlJointMaxLimit(NrOfDOFs);
        std::vector<std::string> jointLimitsNames;
        kdl_format_io::jointPosLimitsFromUrdfFile(urdfFile, jointLimitsNames, kdlJointMinLimit, kdlJointMaxLimit);

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
