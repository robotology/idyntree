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

#ifndef KDL_CODYCO_GENERALIZEDJNTMOTIONS_HPP
#define KDL_CODYCO_GENERALIZEDJNTMOTIONS_HPP

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <Eigen/Core>

namespace KDL
{
namespace CoDyCo
{
    /**
     * @brief Class for representing velocities or accelerations of a floating base robot.
     *
     * This class represents the floating base equivalent of a
     *        KDL::JntArray containing joint velocities or accelerations.
     *
     * It is composed by a KDL::Twist object,
     * representing the base velocity/acceleration of the base link frame
     * with respect to the world frame (H_w_b), and a KDL::JntArray object,
     * representing the velocities/accelerations of the joints of the floating base robot.
     *
     * \note Add details on where the base velocities are expressed
     */

    class GeneralizedJntMotions
    {
    public:
        KDL::Twist base_twist;
        KDL::JntArray jnt;

        /**
         * Base constructor of GeneralizedJntMotions class
         */
        GeneralizedJntMotions();

        /**
         * Constructor of the GeneralizedJntMotions class
         *
         * @param nrOfDOFs number of internal dofs (i.e. size of jnt_pos attribute)
         */
        GeneralizedJntMotions(const int nrOfDOFs);

        /**
         * Constructor
         *
         * @param _base_pos the base link twist
         * @param _jnt the the joints velocities/accelerations
         */
        GeneralizedJntMotions(const KDL::Twist & _base_twist, const KDL::JntArray & _jnt);

        /** Copy constructor
         * @note Will correctly copy an empty object
         */
        GeneralizedJntMotions(const GeneralizedJntMotions& arg);
        ~GeneralizedJntMotions();

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
        GeneralizedJntMotions& operator = ( const GeneralizedJntMotions& arg);
    };

    typedef GeneralizedJntMotions GeneralizedJntVelocities;
    typedef GeneralizedJntMotions GeneralizedJntAccelerations;

}
}

#endif
