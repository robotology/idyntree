/**
 * Copyright  (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/utils.hpp"
#include "kdl_codyco/treeserialization.hpp"
#include <iostream>
#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>

namespace KDL {
namespace CoDyCo {
    double computeMass(const Tree & tree) {
        
        double total_mass = 0.0;
        
        //create necessary vectors
        SegmentMap::const_iterator root;
        
        root = tree.getRootSegment();
        
        SegmentMap sm = tree.getSegments();
           
        for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            //root has no mass
            if( i != root ) {
               total_mass += i->second.segment.getInertia().getMass();
            }
		}
        
        return total_mass;
    }
    
        
	int JointInvertPolarity(const KDL::Joint & old_joint, const KDL::Frame & old_f_tip, KDL::Joint & new_joint, KDL::Frame & new_f_tip)
	{	
 
        switch(old_joint.getType()) 
        {
            case Joint::RotAxis:
            case Joint::RotX:
            case Joint::RotY:
            case Joint::RotZ:
				//\todo document this
				new_f_tip = KDL::Frame(old_f_tip.M.Inverse(),-(old_f_tip.M.Inverse()*old_joint.JointOrigin()));
                new_joint = Joint(old_joint.getName(),-(old_f_tip.M.Inverse()*old_f_tip.p), -(old_f_tip.M.Inverse()*old_joint.JointAxis()), Joint::RotAxis);
            break;
            case Joint::TransAxis:
            case Joint::TransX:
            case Joint::TransY:
            case Joint::TransZ:
				new_f_tip = KDL::Frame(old_f_tip.M.Inverse(),-(old_f_tip.M.Inverse()*old_f_tip.p));
                new_joint = Joint(old_joint.getName(),-(old_f_tip.M.Inverse()*old_joint.JointOrigin()), -(old_f_tip.M.Inverse()*old_joint.JointAxis()), Joint::TransAxis);
            break;
            case Joint::None:
            default:
				new_f_tip = KDL::Frame(old_f_tip.M.Inverse(),-(old_f_tip.M.Inverse()*old_f_tip.p));
                new_joint = Joint(old_joint.getName(), Joint::None);
            break;
        }
        #ifndef NDEBUG
        std::cerr << "Exiting joint polarity inversion, checking all is working" << std::endl;
        double q = 0.83;
        std::cerr << old_joint.pose(q)*old_f_tip*new_joint.pose(q)*new_f_tip << std::endl;
        #endif 
        return 0;
    }

}
}
