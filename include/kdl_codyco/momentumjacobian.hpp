// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2013  Silvio Traversaro < silvio dot traversaro at iit dot it >


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

#ifndef KDL_CODYCO_MOMENTUM_JACOBIAN_HPP
#define KDL_CODYCO_MOMENTUM_JACOBIAN_HPP

#include <kdl/frames.hpp>
#include <Eigen/Core>

namespace KDL
{
namespace CoDyCo
{
    /**
     * This is basically a copy of the KDL::Jacobian class, the only difference is that 
     * its columns are Wrenches, while in ordinary Jacobians the columns are Twists. This is useful 
     * for expressing mapping between joint velocities and elements of M^6, for example the spatial
     * momentum.
     * 
     */
    class MomentumJacobian
    {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Matrix<double,6,Eigen::Dynamic> data;
        MomentumJacobian();
        explicit MomentumJacobian(unsigned int nr_of_columns);
        MomentumJacobian(const MomentumJacobian& arg);

        ///Allocates memory for new size (can break realtime behavior)
        void resize(unsigned int newNrOfColumns);

        ///Allocates memory if size of this and argument is different
        MomentumJacobian& operator=(const MomentumJacobian& arg);

        bool operator ==(const MomentumJacobian& arg)const;
        bool operator !=(const MomentumJacobian& arg)const;
        
        friend bool Equal(const MomentumJacobian& a,const MomentumJacobian& b,double eps=epsilon);
        

        ~MomentumJacobian();

        double operator()(unsigned int i,unsigned int j)const;
        double& operator()(unsigned int i,unsigned int j);
        unsigned int rows()const;
        unsigned int columns()const;

        friend void SetToZero(MomentumJacobian& jac);

        friend bool changeRefPoint(const MomentumJacobian& src1, const Vector& base_AB, MomentumJacobian& dest);
        friend bool changeBase(const MomentumJacobian& src1, const Rotation& rot, MomentumJacobian& dest);
        friend bool changeRefFrame(const MomentumJacobian& src1,const Frame& frame, MomentumJacobian& dest);

        Wrench getColumn(unsigned int i) const;
        void setColumn(unsigned int i,const Wrench& t);

        void changeRefPoint(const Vector& base_AB);
        void changeBase(const Rotation& rot);
        void changeRefFrame(const Frame& frame);


    };

    bool changeRefPoint(const MomentumJacobian& src1, const Vector& base_AB, MomentumJacobian& dest);
    bool changeBase(const MomentumJacobian& src1, const Rotation& rot, MomentumJacobian& dest);
    bool changeRefFrame(const MomentumJacobian& src1,const Frame& frame, MomentumJacobian& dest);

}
}

#endif
