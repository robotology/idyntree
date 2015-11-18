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

#ifndef KDL_CODYCO_GENERALIZEDJNTTORQUES_HPP
#define KDL_CODYCO_GENERALIZEDJNTTORQUES_HPP

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <Eigen/Core>

namespace KDL
{
namespace CoDyCo
{
    /**
     * @brief Class for representing generalized torques of a floating base robot.
     *
     * This class represents the floating base equivalent of a
     *        KDL::JntArray containing joint torques.
     *
     * It is composed by a KDL::Wrench object,
     * representing the base wrench acting on the base link frame
     * with respect to the world frame, and a KDL::JntArray object,
     * representing the torques of the joints of the floating base robot.
     *
     * \note Add details on where the base wrench is expressed
     */

    class GeneralizedJntTorques
    {
    public:
        KDL::Wrench base_wrench;
        KDL::JntArray jnt;

        /**
         * Base constructor of GeneralizedJntTorques class
         */
        GeneralizedJntTorques();

        /**
         * Constructor of the GeneralizedJntTorques class
         *
         * @param nrOfDOFs number of internal dofs (i.e. size of jnt attribute)
         */
        GeneralizedJntTorques(const int nrOfDOFs);

        /**
         * Constructor
         *
         * @param _base_wrench the base link wrench
         * @param _jnt the the joints torques
         */
        GeneralizedJntTorques(const KDL::Wrench _base_wrench, const KDL::JntArray _jnt);

        /** Copy constructor
         * @note Will correctly copy an empty object
         */
        GeneralizedJntTorques(const GeneralizedJntTorques& arg);
        ~GeneralizedJntTorques();

        /**
         * \brief Get the number of internal degrees of freedom
         * Return the degrees of freedom of the robot (excluding the 6 of the floating base)
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
        GeneralizedJntTorques& operator = ( const GeneralizedJntTorques& arg);
    };

}
}

#endif
