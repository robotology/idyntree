// Copyright  (C)  2014  Silvio Traversaro <silvio dot traversaro at iit dot it>

// Version: 1.0
// Author: Silvio Traversaro <silvio dot traversaro at iit dot it>
// Maintainer: Silvio Traversaro <silvio dot traversaro at iit dot it>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef KDL_CODYCO_GENERALIZEDJNTPOSITIONS_HPP
#define KDL_CODYCO_GENERALIZEDJNTPOSITIONS_HPP

#ifdef __DEPRECATED
  #warning <generalizedjntpositions.hpp> is deprecated.
#endif

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <Eigen/Core>

namespace KDL
{
namespace CoDyCo
{
    /**
     * @brief Position of a floating base robot.
     *
     * This class represents the floating base equivalent of a
     *        KDL::JntArray containing joint positions.
     *
     * It is composed by a KDL::Frame object,
     * representing the SE(3) rototranslation from the base link frame
     * to the world frame (H_w_b), and a KDL::JntArray object,
     * representing the position of the joints of the floating base robot.
     */

    class GeneralizedJntPositions
    {
    public:
        KDL::Frame base_pos;
        KDL::JntArray jnt_pos;

        /**
         * Base constructor of GeneralizedJntPositions class
         */
        GeneralizedJntPositions();

        /**
         * Constructor of the GeneralizedJntPositions class
         *
         * @param nrOfDOFs number of internal dofs (i.e. size of jnt_pos attribute)
         */
        GeneralizedJntPositions(const int nrOfDOFs);

        /**
         * Constructor
         *
         * @param _base_pos the base link position
         * @param _jnt_pos the position of the joints
         */
        GeneralizedJntPositions(const KDL::Frame & _base_pos, const KDL::JntArray & _jnt_pos);

        /** Copy constructor
         * @note Will correctly copy an empty object
         */
        GeneralizedJntPositions(const GeneralizedJntPositions& arg);
        ~GeneralizedJntPositions();

        /**
         * \brief Get the number of internal degrees of freedom
         * Return the degrees of freedom of the (excluding the 6 of the floating base)
         *
         */
        unsigned int getNrOfDOFs();

        /**
         * Resize the underlying KDL::JntArray
         */
        void setNrOfDOFs(unsigned int newNrOfDOFs);

        /**
         * Copy operator
         */
        GeneralizedJntPositions& operator = ( const GeneralizedJntPositions& arg);

    };

}
}

#endif
