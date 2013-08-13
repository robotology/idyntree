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

namespace KDL { 
namespace CoDyCo {   

/**
 * 
 * Data structure for containing information about internal FT sensors
 * To properly describe an FT sensor, it is necessary: 
 *      * the fixed junction of the FT sensor in the TreeGraph (a name)
 *      * the transformation from the *parent* KDL::Tree link to the the reference
 *        frame of the FT measurement ( H_p_s ) such that given the measure f_s, the 
 *        wrench applied by the child on the parent expressed in the parent frame ( f_p )
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
        
        std::string getName() { return fixed_joint_name; }					
        
        /**
        * For the given current_link, get the wrench excerted on the subgraph
        * as measured by the FT sensor
        */
        KDL::Wrench getWrenchExcertedOnSubGraph(int current_link, const std::vector<KDL::Wrench> & measured_wrenches )
        {
            if( current_link == parent ) {
                return (H_parent_sensor*measured_wrenches[sensor_id]);
            } else {
                //The junction connected to an F/T sensor should be one with 0 DOF
                assert(tree_graph->getLink(child)->getAdjacentJoint(tree_graph->getLink(parent))->joint.getType() == KDL::Joint::None );
                KDL::Frame H_child_parent = tree_graph->getLink(parent)->pose(tree_graph->getLink(child),0.0);
                assert(current_link == child);
                return H_child_parent*(H_parent_sensor*measured_wrenches[sensor_id]);
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
        
        int getChild() const 
        {
            return child;
        }
        
        int getParent() const
        {
            return parent;
        }
    
};

class FTSensorList
{
    private:
        std::vector< std::vector<FTSensor *> > link_FT_sensors;
        std::vector< int > junction_id2ft_sensor_id;
    
    public: 
        std::vector< FTSensor > ft_sensors_vector;
        
        int getNrOfFTSensors()
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
            link_FT_sensors.resize(tree_graph.getNrOfLinks(),std::vector<FTSensor *>(0));
            junction_id2ft_sensor_id.clear();
            junction_id2ft_sensor_id.resize(tree_graph.getNrOfJunctions(),-1);
            
            for(int i=0; i < (int)ft_names.size(); i++ ) {
                KDL::CoDyCo::JunctionMap::const_iterator junction_it = tree_graph.getJunction(ft_names[i]);
                if( junction_it == tree_graph.getInvalidJunctionIterator() ) { link_FT_sensors.clear(); junction_id2ft_sensor_id.clear(); ft_sensors_vector.clear(); return; }
                int parent_id = junction_it->parent->link_nr;
                int child_id = junction_it->child->link_nr;
                int sensor_id = i;
                ft_sensors_vector.push_back(FTSensor(tree_graph,ft_names[i],parent_id,child_id,sensor_id));

                link_FT_sensors[parent_id].push_back(&(ft_sensors_vector[i]));
                link_FT_sensors[child_id].push_back(&(ft_sensors_vector[i]));
                
                junction_id2ft_sensor_id[junction_it->getJunctionIndex()] = sensor_id;
            }
            return;
        }
        
        
        KDL::Wrench getMeasuredWrench(int link_id,  const std::vector< KDL::Wrench > & measured_wrenches)
        {   
            KDL::Wrench ret = KDL::Wrench::Zero();
            for(int i = 0; i < (int)link_FT_sensors[link_id].size(); i++ ) {
                ret += link_FT_sensors[link_id][i]->getWrenchExcertedOnSubGraph(link_id,measured_wrenches);
            }
            return ret;
        }
        
        bool isFTSensor(int junction_index) {
            return ( junction_id2ft_sensor_id[junction_index] >= 0);
        }
        
        int getFTSensorID(int junction_index) {
            return junction_id2ft_sensor_id[junction_index];
        }
        
        std::vector<FTSensor *> getFTSensorsOnLink(int link_index)
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
        
        int getNrOfFTSensorsOnLink(int link_index)
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
        
};

}
}
    
#endif
