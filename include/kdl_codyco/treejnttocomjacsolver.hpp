/*
 *  
 *      Author: Silvio Traversaro
 */

#ifndef _KDL_CODYCO_TREEJNTTOCOMJACSOLVER_HPP_
#define _KDL_CODYCO_TREEJNTTOCOMJACSOLVER_HPP_

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

#include <kdl_codyco/treeserialization.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include <kdl_codyco/undirectedtreesolver.hpp>
#include <kdl_codyco/crba_loops.hpp>

#include <kdl_codyco/momentumjacobian.hpp>

namespace KDL {
namespace CoDyCo {

/**
 * Class for calculating the jacobian of the center of mass
 * and the jacobian of the spatial momentum
 */
class TreeJntToCOMJacSolver: public UndirectedTreeSolver {

private:
    std::vector<RigidBodyInertia> Ic;
    MomentumJacobian h_jac;
    RigidBodyInertia I_com;

    
public:
    explicit TreeJntToCOMJacSolver(const Tree& tree, const TreeSerialization & serialization=TreeSerialization());

    virtual ~TreeJntToCOMJacSolver();

    /*
     * Calculate the floating base jacobian for the center of mass velocity, expressed with respect 
     * to the orientation of the base link and in the center of mass, to obtain the proper 3d velocity 
     * of the center of mass.
     *
     * Only the first three rows are the velocity of the center of mass, the remaing three rows
     * are the jacobian of the average angular velocity of the robot, as defined in:
     * @article{Orin2013,
     *      author = {Orin, David E. and Goswami, Ambarish and Lee, Sung-Hee},
     *      doi = {10.1007/s10514-013-9341-4},
     *      issn = {0929-5593},
     *      journal = {Autonomous Robots},
     *      title = {{Centroidal dynamics of a humanoid robot}},
     *      volume = {35},
     *      year = {2013}
     * }
     * 
     */
    int JntToCOMJac(const JntArray& q_in, Jacobian& jac);
    
    /*
     * Calculate the floating base jacobian for the center of mass velocity, expressed with respect 
     * to the orientation of the base link and in the center of mass.
     *
     * The used O(n) algorithm is based on che CRBA algorithm as explained in:
     * @article{Orin2013,
     *      author = {Orin, David E. and Goswami, Ambarish and Lee, Sung-Hee},
     *      doi = {10.1007/s10514-013-9341-4},
     *      issn = {0929-5593},
     *      journal = {Autonomous Robots},
     *      title = {{Centroidal dynamics of a humanoid robot}},
     *      volume = {35},
     *      year = {2013}
     * }
     * 
     */
    int JntToMomentumJac(const JntArray& q_in, MomentumJacobian& jac);

};

}//End of namespace
}

#endif /* TREEJNTTOJACSOLVER_H_ */