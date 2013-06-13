/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
 
#ifndef KDL_CODYCO_UTILS_HPP
#define KDL_CODYCO_UTILS_HPP

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

namespace KDL {
namespace CoDyCo {

    /**
     * Compute the total mass of a KDL::Tree object
     * 
     * @param tree the KDL::Tree object
     * @return the mass of the robot, < 0 if something failed
     * 
     */
    double computeMass(const Tree & tree);
    
    /**
     * Perform the kinetic phase of the RNEA algorithm 
     * 
     * \warning Basic function designed for use inside the solver, some no
     *          error checking on input/output parameters is done
     */
     /*
     int rneaKinematicLoop(const JntArray &q, 
                       const JntArray &q_dot,
                       const JntArray &q_dotdot,  
                       const Twist& base_velocity, 
                       const Twist& base_acceleration, 
                       const std::vector<int>& visit_order,
                       const std::vector<SegmentMap::const_iterator>& index2segment,
                       const std::vector<int>& parent, 
                       const std::vector<int>& link2joint,
                             std::vector<Frame>& X,
                             std::vector<Twist>& S,
                             std::vector<Twist>& v,
                             std::vector<Twist>& a,
                             std::vector<Wrench>& f= std::vector<Wrench>(0)
                       );
                       */
                       
    /**
     * Perform the dynamics phase of the RNEA algorithm, where the kinematic
     * quantites (kin_X) where calculate with a loop with a different base
     * 
     * \warning Basic function designed for use inside the solver, some no
     *          error checking on input/output parameters is done
     */
     int rneaDynamicLoop(
                       const std::vector<int>& visit_order,
                       const std::vector<SegmentMap::const_iterator>& index2segment,
                       const std::vector<int>& dyn_parent, 
                       const std::vector<int>& link2joint,
                       const std::vector<int>& kin_parent,
                            std::vector<Frame>& kinX,
                             std::vector<Twist>& S,
                             std::vector<Twist>& v,
                             std::vector<Twist>& a,
                             std::vector<Wrench>& f
                       );                   
    
    /**
     * Project a Vector from frame of index j to frame of index i, assuming
     * that links i and j are connected
     * 
     * \note if NDEBUG is not defined, execute checks on the input data
     * 
     * @return X[j]*arg if parent(j) == i, X[i].Inverse()*arg if parent(i) == j
     */
    //Vector project(const Vector & arg,const int i,const int j,const std::vector<Frame>& X,const std::vector<int>& parent);

    /**
     * Project a Wrench from frame of index j to frame of index i, assuming
     * that links i and j are connected
     * 
     * \note if NDEBUG is not defined, execute checks on the input data
     * 
     * @return X[j]*arg if parent(j) == i, X[i].Inverse()*arg if parent(i) == j
     */
    //Wrench project(const Frame & arg,const int i,const int j,const std::vector<Frame>& X,const std::vector<int>& parent);

    /**
     * Project a Twist from frame of index j to frame of index i, assuming
     * that links i and j are connected
     * 
     * \note if NDEBUG is not defined, execute checks on the input data
     * 
     * @return X[j]*arg if parent(j) == i, X[i].Inverse()*arg if parent(i) == j
     */
    //Vector project(const Wrench & arg,const int i,const int j,const std::vector<Frame>& X,const std::vector<int>& parent);

                       
}
}  



#endif 
