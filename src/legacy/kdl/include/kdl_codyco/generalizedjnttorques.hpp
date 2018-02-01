// Copyright  (C)  2014  Silvio Traversaro <silvio dot traversaro at iit dot it>

/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef KDL_CODYCO_GENERALIZEDJNTTORQUES_HPP
#define KDL_CODYCO_GENERALIZEDJNTTORQUES_HPP

#ifdef __DEPRECATED
  #warning <generalizedjnttorques.hpp> is deprecated.
#endif

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
