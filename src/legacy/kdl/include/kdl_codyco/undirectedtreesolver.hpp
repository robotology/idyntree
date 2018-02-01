/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef UNDIRECTED_TREE_SOLVER_H
#define UNDIRECTED_TREE_SOLVER_H

#ifdef __DEPRECATED
  #warning <undirectedtreesolver.hpp> is deprecated.
#endif

#include <kdl/tree.hpp>
#include "undirectedtree.hpp"
#include "treeserialization.hpp"

namespace KDL
{
namespace CoDyCo
{
    /**
    * \brief This is the base class for all the Tree solvers (kinematics,
     *  dynamics, COM, ... ) that use the UndirectedTree object.
     *  If a solver uses this base class, then for that solver
     *  it will be possible to define a custom serialization for joints
     *  and links using a KDL::CoDyCo::TreeSerialization object.
     * It will also be possible to change online the base of the
     * tree using the changeBase method.
     */
    class UndirectedTreeSolver
    {
        protected:
            const UndirectedTree undirected_tree;
            Traversal traversal;

        public:
            UndirectedTreeSolver(const Tree & tree_arg,
                                 const TreeSerialization & serialization_arg):
                                undirected_tree(tree_arg,serialization_arg)
            {
                undirected_tree.compute_traversal(traversal);
                assert(undirected_tree.check_consistency(traversal) == 0);
            };

            ~UndirectedTreeSolver() {};

            /**
             * Change the link used as the base one in the solver
             *
             * @param[in] new_base_name the name of the new base
             * @return true if all went well, false otherwise
             */
            bool changeBase(const std::string new_base_name)
            {
               int ret = undirected_tree.compute_traversal(traversal,new_base_name);
               if( ret != 0 ) { return false; }

               return true;
            }

            /**
             * Change the link used as the base one in the solver
             *
             * @param[in] new_base_id the ID of the new base (as specified in getSerialization())
             * @return true if all went well, false otherwise
             */
            bool changeBase(const int new_base_id)
            {
                if( new_base_id < 0 || new_base_id >= (int)undirected_tree.getNrOfLinks() ) {
                    return false;
                }

               int ret = undirected_tree.compute_traversal(traversal,new_base_id);
               if( ret != 0 ) { return false; }

               return true;
            }


            /**
             * Return the KDL::CoDyCo::UndirectedTree object used for the solver.
             * The KDL::CoDyCo::UndirectedTree object is generated in the constructor
             * compining the information from the KDL::Tree and (if present) the
             * TreeSerialization object.
             *
             * @return the TreeSerialization object
             */
            const UndirectedTree & getUndirectedTree() const
            {
                return undirected_tree;
            }

            /**
             * Return the KDL::CoDyCo::TreeSerialization object used for the solver.
             * If it was not specified in the constructor, it is the default one of
             * input KDL::Tree (i.e. if tree is the input tree: TreeSerialization(tree))
             *
             * @return the TreeSerialization object
             */
            const TreeSerialization getSerialization() const
            {
                return undirected_tree.getSerialization();
            }
    };

}
}

#endif
