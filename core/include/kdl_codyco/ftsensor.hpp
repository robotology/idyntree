/**
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#ifndef _KDL_CODYCO_FT_SENSOR_
#define _KDL_CODYCO_FT_SENSOR_

#ifndef NDEBUG
#include <iostream>
#include <kdl/frames_io.hpp>
#endif

#include <sstream>
#include <kdl_codyco/treefksolverpos_iterative.hpp>

namespace KDL {
namespace CoDyCo {

/**
 *
 * \todo fix copy operator
 *
 * Data structure for containing information about internal FT sensors
 * To properly describe an FT sensor, it is necessary:
 *      * the fixed junction of the FT sensor in the UndirectedTree (a name)
 *      * the transformation from the *child* KDL::Tree link to the the reference
 *        frame of the FT measurement ( H_c_s ) such that given the measure f_s, the
 *        wrench applied by the child on the parent (unless the proper parameter in the constructor is used)
 *        expressed in the child frame ( f_c ) is given by f_c = H_c_s f_s
 *
 */
class FTSensor
{
    private:
        const KDL::CoDyCo::UndirectedTree * p_undirected_tree;
        std::string fixed_joint_name;
        KDL::Frame H_child_sensor;
        int parent;
        int child;
        int junction_id;
        int sensor_id;

        bool measured_wrench_is_from_parent_to_child;

    public:
        FTSensor()
        {
            p_undirected_tree =0;
            H_child_sensor = KDL::Frame::Identity();
            parent = child = junction_id = sensor_id = -1;
            measured_wrench_is_from_parent_to_child = false;
            fixed_joint_name = "FT_SENSOR_ERROR";
        }

        FTSensor(const KDL::CoDyCo::UndirectedTree & _undirected_tree,
                const std::string _fixed_joint_name,
                const KDL::Frame _H_child_sensor,
                const int _parent,
                const int _child,
                const int _sensor_id,
                const bool wrench_is_parent_applied_to_child = false
                ) :
                p_undirected_tree(&_undirected_tree),
                fixed_joint_name(_fixed_joint_name),
                H_child_sensor(_H_child_sensor),
                parent(_parent),
                child(_child),
                sensor_id(_sensor_id),
                measured_wrench_is_from_parent_to_child(wrench_is_parent_applied_to_child)
                {junction_id = p_undirected_tree->getLink(child)->getAdjacentJoint(p_undirected_tree->getLink(parent))->getJunctionIndex();}


        ~FTSensor() {}

        std::string getName() const { return fixed_joint_name; }

        /**
        * For the given current_link, get the wrench excerted on the subgraph
        * as measured by the FT sensor
        */
        KDL::Wrench getWrenchExcertedOnSubGraph(int current_link,
                                             const std::vector<KDL::Wrench> & measured_wrenches ) const
        {
            if( current_link == parent ) {
                assert( p_undirected_tree->getLink(child)->getAdjacentJoint(p_undirected_tree->getLink(parent))->getJoint().getType() == KDL::Joint::None );
                KDL::Frame H_child_parent = p_undirected_tree->getLink(parent)->pose(p_undirected_tree->getLink(child),0.0);
                assert(current_link == parent);
                if( measured_wrench_is_from_parent_to_child ) {
                    return -(H_child_parent.Inverse()*(H_child_sensor*measured_wrenches[sensor_id]));
                } else {
                    return (H_child_parent.Inverse()*(H_child_sensor*measured_wrenches[sensor_id]));
                }
            } else {
                //The junction connected to an F/T sensor should be one with 0 DOF
                if( measured_wrench_is_from_parent_to_child ) {
                    return (H_child_sensor*measured_wrenches[sensor_id]);
                } else {
                    return -(H_child_sensor*measured_wrenches[sensor_id]);
                }
            }
        }

        KDL::Frame getH_parent_sensor() const
        {
            assert(p_undirected_tree->getLink(child)->getAdjacentJoint(p_undirected_tree->getLink(parent))->getJoint().getType() == KDL::Joint::None );
            assert(p_undirected_tree->getLink(parent)->getAdjacentJoint(p_undirected_tree->getLink(child))->getJoint().getType() == KDL::Joint::None );
            KDL::Frame H_child_parent = p_undirected_tree->getLink(parent)->pose(p_undirected_tree->getLink(child),0.0);
            return H_child_parent.Inverse()*H_child_sensor;
        }

        KDL::Frame getH_child_sensor() const
        {
            return H_child_sensor;
        }

        KDL::Frame getH_link_sensor(int link_id) const
        {
            assert(link_id == parent || link_id == child);
            if( link_id == parent ) {
                return getH_parent_sensor();
            } else if ( link_id == child ) {
                return getH_child_sensor();
            } else {
                assert(false);
                return KDL::Frame();
            }
        }

        int getChild() const
        {
            return child;
        }

        int getParent() const
        {
            return parent;
        }

        int getJunctionID() const
        {
            return junction_id;
        }

        int getID() const
        {
            return sensor_id;
        }

        int isWrenchAppliedFromParentToChild() const
        {
            return measured_wrench_is_from_parent_to_child;
        }

};

class FTSensorList
{
    private:
        std::vector< std::vector< FTSensor > > link_FT_sensors;
        std::vector< int > junction_id2ft_sensor_id;

    public:
        std::vector< FTSensor > ft_sensors_vector;

        int getNrOfFTSensors() const
        {
            return ft_sensors_vector.size();
        }

        FTSensorList()
        {
            ft_sensors_vector.clear();
            link_FT_sensors.clear();
            junction_id2ft_sensor_id.clear();
        }

        FTSensorList(const KDL::CoDyCo::UndirectedTree & undirected_tree,
                     const std::vector<std::string> & ft_names,
                     const std::vector<bool> & is_measure_direction_child_to_parent)
        {
            KDL::CoDyCo::TreeFkSolverPos_iterative pos_solver(undirected_tree.getTree(),undirected_tree.getSerialization());
            KDL::CoDyCo::GeneralizedJntPositions pos(undirected_tree.getNrOfDOFs());


            link_FT_sensors.clear();
            std::vector<FTSensor> empty_ft_list;
            link_FT_sensors.resize(undirected_tree.getNrOfLinks(),empty_ft_list);
            junction_id2ft_sensor_id.clear();
            junction_id2ft_sensor_id.resize(undirected_tree.getNrOfJunctions(),-1);

            for(int i=0; i < (int)ft_names.size(); i++ ) {
                KDL::CoDyCo::JunctionMap::const_iterator junction_it = undirected_tree.getJunction(ft_names[i]);
                if( junction_it == undirected_tree.getInvalidJunctionIterator() ) { link_FT_sensors.clear(); junction_id2ft_sensor_id.clear(); ft_sensors_vector.clear(); return; }
                int parent_id = junction_it->getParentLink()->getLinkIndex();
                std::string parent_name = junction_it->getParentLink()->getName();
                int child_id = junction_it->getChildLink()->getLinkIndex();
                int sensor_id = i;
#ifndef NDEBUG
                //std::cout << "Adding FT sensor " << i << "That connects " << parent_id << " and " << child_id << std::endl;
#endif
                // \todo TODO fully support the URDF/SDF ft sensor spec
                KDL::Frame H_child_sensor = KDL::Frame::Identity();
                bool is_measure_direction_parent_to_child = !(is_measure_direction_child_to_parent[i]);
                ft_sensors_vector.push_back(FTSensor(undirected_tree,ft_names[i],H_child_sensor,parent_id,child_id,sensor_id,is_measure_direction_parent_to_child));
#ifndef NDEBUG
                //std::cout << "that have name " << ft_sensors_vector[i]->getName() << std::endl;
#endif

                link_FT_sensors[parent_id].push_back((ft_sensors_vector[i]));
                link_FT_sensors[child_id].push_back((ft_sensors_vector[i]));

                junction_id2ft_sensor_id[junction_it->getJunctionIndex()] = sensor_id;
#ifndef NDEBUG
                //std::cout << toString() << std::endl;
#endif
            }
            return;
        }

        ~FTSensorList()
        {
            /*
            for(int ft_id=0; ft_id < ft_sensors_vector.size(); ft_id++ ) {
                if(ft_sensors_vector[ft_id]) { delete ft_sensors_vector[ft_id]; ft_sensors_vector[ft_id] = 0; }
            }*/
            //ft_sensors_vector.resize(0);
        }

        KDL::Wrench getMeasuredWrench(int link_id,  const std::vector< KDL::Wrench > & measured_wrenches) const
        {
            KDL::Wrench ret = KDL::Wrench::Zero();
            for(int i = 0; i < (int)link_FT_sensors[link_id].size(); i++ ) {
                ret += link_FT_sensors[link_id][i].getWrenchExcertedOnSubGraph(link_id,measured_wrenches);
            }
            return ret;
        }

        bool isFTSensor(int junction_index) const {
            return ( junction_id2ft_sensor_id[junction_index] >= 0);
        }

        int getFTSensorID(int junction_index) const {
            return junction_id2ft_sensor_id[junction_index];
        }

        std::vector< FTSensor> getFTSensorsOnLink(int link_index) const
        {
            /*
            if( link_index < 0 || link_index >= .getNrOfLinks() ) {
                /// \todo add verbose option
                #ifndef NDEBUG
                std::cerr << "FTSensorList::getFTSensorsOnLink error: link index out of bounds" << std::endl;
                #endif
                return std::vector<FTSensor *>(0);
            }*/
            return link_FT_sensors[link_index];
        }

        int getNrOfFTSensorsOnLink(int link_index) const
        {
            if( getNrOfFTSensors() == 0 ) {
                return 0;
            }

            /*
            if( link_index < 0 || link_index >= ->getNrOfLinks() ) {
                /// \todo add verbose option
                #ifndef NDEBUG
                std::cerr << "FTSensorList::getNrOfFTSensorsOnLink error: link index out of bounds" << std::endl;
                #endif
                return 0;
            }*/
            return link_FT_sensors[link_index].size();
        }

        /**
         * Given a std::vector<KDL::Wrench> calculate by the dynamic phase of the RNEA, and the corresponding
         * Traversal, get the consequent sensor wrench
         *
         */
        KDL::Wrench estimateSensorWrenchFromRNEA(int ft_sensor_id, KDL::CoDyCo::Traversal & dynamic_traversal, std::vector<KDL::Wrench> f)
        {
#ifndef NDEBUG
            std::cerr << "estimateSensorWrenchFromRNEA ft_sensor_id " << ft_sensor_id << std::endl;
            std::cerr << "is ft_sensor " << ft_sensors_vector[ft_sensor_id].getName() << " ( " << ft_sensor_id << " )  that connects "
            << ft_sensors_vector[ft_sensor_id].getParent() << " and " << ft_sensors_vector[ft_sensor_id].getChild() << std::endl;
            std::cerr << "H_child_sensor: " << ft_sensors_vector[ft_sensor_id].getH_child_sensor() << std::endl;
            std::cerr << "H_parent_sensor: " << ft_sensors_vector[ft_sensor_id].getH_parent_sensor() << std::endl;
#endif
            int child_id = ft_sensors_vector[ft_sensor_id].getChild();
            int parent_id = ft_sensors_vector[ft_sensor_id].getParent();

            double sign;
            if( ft_sensors_vector[ft_sensor_id].isWrenchAppliedFromParentToChild() ) {
                sign = +1.0;
            } else {
                sign = -1.0;
            }

            if(  parent_id == dynamic_traversal.getParentLink(child_id)->getLinkIndex()  ) {
                 return sign*ft_sensors_vector[ft_sensor_id].getH_child_sensor().Inverse(f[child_id]);
            } else if (child_id == dynamic_traversal.getParentLink(parent_id)->getLinkIndex() ) {
                 return -sign*ft_sensors_vector[ft_sensor_id].getH_parent_sensor().Inverse(f[parent_id]);
            } else {
                assert(false);
                return KDL::Wrench::Zero();
            }
        }

        std::string toString() const
        {
            std::stringstream ss;
            for(int i=0; i < (int)ft_sensors_vector.size(); i++ ) {
                int parent_id = ft_sensors_vector[i].getParent();
                int child_id = ft_sensors_vector[i].getChild();
                ss << "FT sensor " << i << " ( " << ft_sensors_vector[i].getName() << " ) " << " connects links " << parent_id << " and " << child_id << std::endl;
                ss << "Link " << parent_id << "has sensor " << link_FT_sensors[parent_id][0].getID() << std::endl;
                ss << "Link " << child_id << "has sensor " << link_FT_sensors[child_id][0].getID() << std::endl;
            }
            return ss.str();
        }

};

}
}

#endif
