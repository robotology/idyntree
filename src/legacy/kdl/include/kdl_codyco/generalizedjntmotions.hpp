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

#ifndef KDL_CODYCO_GENERALIZEDJNTMOTIONS_HPP
#define KDL_CODYCO_GENERALIZEDJNTMOTIONS_HPP

#ifdef __DEPRECATED
  #warning <generalizedjntmotions.hpp> is deprecated.
#endif

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
