/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "kdl_codyco/utils.hpp"

namespace KDL {

    double computeMass(const Tree & tree) {
        
        double total_mass = 0.0;
        
        //create necessary vectors
        SegmentMap::const_iterator root;
        
        tree.getRootSegment(root);
        
        SegmentMap sm = tree.getSegments();
           
        for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            //root has no mass
            if( i != root ) {
               total_mass += i->second.segment.getInertia().getMass();
            }
		}
        
        return total_mass;
    }
    
}
