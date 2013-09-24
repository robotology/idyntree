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
#endif

#include <sstream>

namespace KDL { 
namespace CoDyCo {   

/**
 * 
 * \todo fix copy operator
 * 
 * Data structure for containing information about internal FT sensors
 * To properly describe an FT sensor, it is necessary: 
 *      * the fixed junction of the FT sensor in the TreeGraph (a name)
 *      * the transformation from the *parent* KDL::Tree link to the the reference
 *        frame of the FT measurement ( H_p_s ) such that given the measure f_s, the 
 *        wrench applied by the parent on the child expressed in the parent frame ( f_p )
 *        is given by f_p = H_p_s f_s
 *         
 */
class FTSensor
{
    private:
        const KDL::CoDyCo::TreeGraph * tree_graph;
        std::string fixed_joint_name;
        KDL::Frame H_parent_sensor;
        int parent;
        int child;
        int sensor_id;
      
        
    public:
        FTSensor(const KDL::CoDyCo::TreeGraph & _tree_graph, 
                const std::string _fixed_joint_name,
                const int _parent,
                const int _child,
                const int _sensor_id) : 
                tree_graph(&_tree_graph),
                fixed_joint_name(_fixed_joint_name),
                H_parent_sensor(KDL::Frame::Identity()),
                parent(_parent),
                child(_child),
                sensor_id(_sensor_id) {}
        
        FTSensor(const KDL::CoDyCo::TreeGraph & _tree_graph, 
                const std::string _fixed_joint_name,
                const KDL::Frame _H_parent_sensor,
                const int _parent,
                const int _child,
                const int _sensor_id) : 
                tree_graph(&_tree_graph),
                fixed_joint_name(_fixed_joint_name),
                H_parent_sensor(_H_parent_sensor),
                parent(_parent),
                child(_child),
                sensor_id(_sensor_id) {}
                        
        ~FTSensor() {}
        
        std::string getName() const { return fixed_joint_name; }					
        
        /**
        * For the given current_link, get the wrench excerted on the subgraph
        * as measured by the FT sensor
        */
        KDL::Wrench getWrenchExcertedOnSubGraph(int current_link, const std::vector<KDL::Wrench> & measured_wrenches ) const
        {
            if( current_link == parent ) {
                return -(H_parent_sensor*measured_wrenches[sensor_id]);
            } else {
                //The junction connected to an F/T sensor should be one with 0 DOF
                assert(tree_graph->getLink(child)->getAdjacentJoint(tree_graph->getLink(parent))->joint.getType() == KDL::Joint::None );
                KDL::Frame H_child_parent = tree_graph->getLink(parent)->pose(tree_graph->getLink(child),0.0);
                assert(current_link == child);
                return (H_child_parent*(H_parent_sensor*measured_wrenches[sensor_id]));
            }
        }
        
        KDL::Frame getH_parent_sensor() const 
        {
            return H_parent_sensor;
        }
        
        KDL::Frame getH_child_sensor() const 
        {
            assert(tree_graph->getLink(child)->getAdjacentJoint(tree_graph->getLink(parent))->joint.getType() == KDL::Joint::None );
            assert(tree_graph->getLink(parent)->getAdjacentJoint(tree_graph->getLink(child))->joint.getType() == KDL::Joint::None );
            KDL::Frame H_child_parent = tree_graph->getLink(parent)->pose(tree_graph->getLink(child),0.0);
            return H_child_parent*H_parent_sensor;
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
        
        int getID() const 
        {
            return sensor_id;
        }
    
};

class FTSensorList
{
    private:
        std::vector< std::vector<const FTSensor *> > link_FT_sensors;
        std::vector< int > junction_id2ft_sensor_id;
    
    public: 
        std::vector< FTSensor * > ft_sensors_vector;
        
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
    
        FTSensorList(const KDL::CoDyCo::TreeGraph & tree_graph, const std::vector<std::string> & ft_names)
        {
            link_FT_sensors.clear();
            link_FT_sensors.resize(tree_graph.getNrOfLinks(),std::vector<const FTSensor *>(0));
            junction_id2ft_sensor_id.clear();
            junction_id2ft_sensor_id.resize(tree_graph.getNrOfJunctions(),-1);
            
            for(int i=0; i < (int)ft_names.size(); i++ ) {
                KDL::CoDyCo::JunctionMap::const_iterator junction_it = tree_graph.getJunction(ft_names[i]);
                if( junction_it == tree_graph.getInvalidJunctionIterator() ) { link_FT_sensors.clear(); junction_id2ft_sensor_id.clear(); ft_sensors_vector.clear(); return; }
                int parent_id = junction_it->parent->link_nr;
                int child_id = junction_it->child->link_nr;
                int sensor_id = i;
#ifndef NDEBUG
                //std::cout << "Adding FT sensor " << i << "That connects " << parent_id << " and " << child_id << std::endl;
#endif 
                ft_sensors_vector.push_back(new FTSensor(tree_graph,ft_names[i],parent_id,child_id,sensor_id));
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
        
        
        KDL::Wrench getMeasuredWrench(int link_id,  const std::vector< KDL::Wrench > & measured_wrenches) const
        {   
            KDL::Wrench ret = KDL::Wrench::Zero();
            for(int i = 0; i < (int)link_FT_sensors[link_id].size(); i++ ) {
                ret += link_FT_sensors[link_id][i]->getWrenchExcertedOnSubGraph(link_id,measured_wrenches);
            }
            return ret;
        }
        
        bool isFTSensor(int junction_index) const {
            return ( junction_id2ft_sensor_id[junction_index] >= 0);
        }
        
        int getFTSensorID(int junction_index) const {
            return junction_id2ft_sensor_id[junction_index];
        }
        
        std::vector<const FTSensor *> getFTSensorsOnLink(int link_index) const
        {
            /*
            if( link_index < 0 || link_index >= tree_graph.getNrOfLinks() ) {
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
            if( link_index < 0 || link_index >= tree_graph->getNrOfLinks() ) {
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
            //std::cerr << "estimateSensorWrenchFromRNEA ft_sensor_id " << ft_sensor_id << std::endl;
            //std::cerr << "is ft_sensor " << ft_sensors_vector[ft_sensor_id]->getName() << " ( " << ft_sensor_id << " )  that connects "
            //<< ft_sensors_vector[ft_sensor_id]->getParent() << " and " << ft_sensors_vector[ft_sensor_id]->getChild() << std::endl;
#endif
            int child_id = ft_sensors_vector[ft_sensor_id]->getChild();
            int parent_id = ft_sensors_vector[ft_sensor_id]->getParent();
            
            if(  parent_id == dynamic_traversal.parent[child_id]->getLinkIndex()  ) {
                 return ft_sensors_vector[ft_sensor_id]->getH_child_sensor().Inverse(f[child_id]);
            } else if (child_id == dynamic_traversal.parent[parent_id]->getLinkIndex() ) {
                 return -ft_sensors_vector[ft_sensor_id]->getH_parent_sensor().Inverse(f[parent_id]);
            } else {
                assert(false);
                return KDL::Wrench::Zero();
            }
        }
        
        std::string toString() const
        {
            std::stringstream ss;
            for(int i=0; i < ft_sensors_vector.size(); i++ ) {
                int parent_id = ft_sensors_vector[i]->getParent();
                int child_id = ft_sensors_vector[i]->getChild();
                ss << "FT sensor " << i << " ( " << ft_sensors_vector[i]->getName() << " ) " << " connects links " << parent_id << " and " << child_id << std::endl;
                ss << "Link " << parent_id << "has sensor " << link_FT_sensors[parent_id][0]->getID() << std::endl;
                ss << "Link " << child_id << "has sensor " << link_FT_sensors[child_id][0]->getID() << std::endl;
            }
            return ss.str();
        }
        
};

}
}
    
#endif
