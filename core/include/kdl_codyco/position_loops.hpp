/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef KDL_CODYCO_POSITION_LOOPS_HPP
#define KDL_CODYCO_POSITION_LOOPS_HPP

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include "kdl_codyco/generalizedjntpositions.hpp"

namespace KDL {
namespace CoDyCo {

   /**
    * Loop for calculating, given a UndirectedTree and a Traversal, the Frame
    * between a distal link and a proximal link (with the proximal link ancestor (given the Traversal)
    * of the distal one). The function return the frame of the distal with respect to the base ( ${}^{base}H_{distal}$)
    *
    * \warning Basic function designed for use inside the solver, some the
    *          error checking on input/output parameters is not guaranteed
    *
    * \todo add version of the loop with using link names or references
    *
    */
   int getFrameLoop(const UndirectedTree & undirected_tree,
                const KDL::JntArray &q,
                const Traversal & traversal,
                const int proximal_link_index,
                const int distal_link_index,
                Frame & frame_proximal_distal);

   /**
    * Loop for calculating, given a UndirectedTree and a Traversal, the Frame
    * of alink with respect to the world reference frame ( ${}^{world}H_{link}$)
    *
    * \warning Basic function designed for use inside the solver, some the
    *          error checking on input/output parameters is not guaranteed
    *
    * \todo add version of the loop with using link names or references
    *
    */
   int getWorldFrameLoop(const UndirectedTree & undirected_tree,
                          const KDL::CoDyCo::GeneralizedJntPositions &q,
                          const Traversal & traversal,
                          const int distal_link_index,
                          Frame & frame_world_link);


   /**
    * Loop for calculating, given a UndirectedTree and a Traversal, the Frame
    * between a distal link and a proximal link (with the proximal link ancestor (given the Traversal)
    * of the distal one). The function return the frame of the distal with respect to the base ( ${}^{base}H_{distal}$)
    *
    * \warning Basic function designed for use inside the solver, some the
    *          error checking on input/output parameters is not guaranteed
    *
    * \todo add version of the loop with using link names or references
    *
    */
    Frame getFrameLoop(const UndirectedTree & undirected_tree,
                       const KDL::JntArray &q,
                       const Traversal & traversal,
                       const int proximal_link_index,
                       const int distal_link_index);


   /**
    * Loop for calculating, given a UndirectedTree and a Traversal,
    * the reference frames of every link with respect to the base link
    * \warning Basic function designed for use inside the solver, some the
    *          error checking on input/output parameters is not guaranteed
    *
    *
    */
    int getFramesLoop(const UndirectedTree & undirected_tree,
                      const KDL::JntArray &q,
                      const Traversal & traversal,
                      std::vector<Frame> & X_base);

   /**
    * Loop for calculating, given a UndirectedTree and a Traversal,
    * the reference frames of every link with respect to the world link ( ${}^{world}H_{link}$)
    * \warning Basic function designed for use inside the solver, some the
    *          error checking on input/output parameters is not guaranteed
    *
    *
    */
    int getWorldFramesLoop(const UndirectedTree & undirected_tree,
                           const KDL::CoDyCo::GeneralizedJntPositions &q,
                           const Traversal & traversal,
                           std::vector<Frame> & X_base);
}
}



#endif
