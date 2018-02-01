// Copyright  (C)  2013  Silvio Traversaro <silvio dot traversaro at iit dot it>
// Copyright  (C)  2009  Dominick Vanthienen <dominick dot vanthienen at mech dot kuleuven dot be>

/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <kdl_codyco/floatingjntspaceinertiamatrix.hpp>

#include <kdl_codyco/regressor_utils.hpp>

namespace KDL
{
namespace CoDyCo
{
    using namespace Eigen;

    FloatingJntSpaceInertiaMatrix::FloatingJntSpaceInertiaMatrix()
    {
    }

    FloatingJntSpaceInertiaMatrix::FloatingJntSpaceInertiaMatrix(int _size):
        data(_size, _size)
    {
        data.setZero();
    }


    FloatingJntSpaceInertiaMatrix::FloatingJntSpaceInertiaMatrix(const FloatingJntSpaceInertiaMatrix& arg):
        data(arg.data)
    {
    }

    FloatingJntSpaceInertiaMatrix& FloatingJntSpaceInertiaMatrix::operator = (const FloatingJntSpaceInertiaMatrix& arg)
    {
        data=arg.data;
        return *this;
    }


    FloatingJntSpaceInertiaMatrix::~FloatingJntSpaceInertiaMatrix()
    {
    }

    void FloatingJntSpaceInertiaMatrix::resize(unsigned int newSize)
    {
        data.resize(newSize,newSize);
        data.setZero();
    }

    double FloatingJntSpaceInertiaMatrix::operator()(unsigned int i,unsigned int j)const
    {
        return data(i, j);
    }

    double& FloatingJntSpaceInertiaMatrix::operator()(unsigned int i,unsigned int j)
    {
        return data(i, j);
    }

    unsigned int FloatingJntSpaceInertiaMatrix::rows()const
    {
        return data.rows();
    }

    unsigned int FloatingJntSpaceInertiaMatrix::columns()const
    {
        return data.cols();
    }


    void Add(const FloatingJntSpaceInertiaMatrix& src1,const FloatingJntSpaceInertiaMatrix& src2,FloatingJntSpaceInertiaMatrix& dest)
    {
        dest.data=src1.data+src2.data;
    }

    void Subtract(const FloatingJntSpaceInertiaMatrix& src1,const FloatingJntSpaceInertiaMatrix& src2,FloatingJntSpaceInertiaMatrix& dest)
    {
        dest.data=src1.data-src2.data;
    }

    void Multiply(const FloatingJntSpaceInertiaMatrix& src,const double& factor,FloatingJntSpaceInertiaMatrix& dest)
    {
        dest.data=factor*src.data;
    }

    void Divide(const FloatingJntSpaceInertiaMatrix& src,const double& factor,FloatingJntSpaceInertiaMatrix& dest)
    {
        dest.data=src.data/factor;
    }

    void Multiply(const FloatingJntSpaceInertiaMatrix& src, const Twist & base_vel, const JntArray& ddq, Wrench & base_wrench, JntArray& tau)
    {
        //Calculate output wrench
        Eigen::Matrix<double,6,1> wrench_eigen;
        wrench_eigen = src.data.block(0,0,6,6)*toEigen(base_vel) + src.data.block(0,6,6,src.data.cols()-6)*ddq.data;
        base_wrench = toKDLWrench(wrench_eigen);

        //Calculate output torque
        tau.data= src.data.block(6,0,src.rows()-6,6)*toEigen(base_vel) + src.data.block(6,6,src.data.rows()-6,src.data.cols()-6)*ddq.data;
    }

    void SetToZero(FloatingJntSpaceInertiaMatrix& mat)
    {
        mat.data.setZero();
    }

    bool Equal(const FloatingJntSpaceInertiaMatrix& src1, const FloatingJntSpaceInertiaMatrix& src2,double eps)
    {
        if(src1.rows()!=src2.rows()||src1.columns()!=src2.columns())
            return false;
        return src1.data.isApprox(src2.data,eps);
    }

    bool operator==(const FloatingJntSpaceInertiaMatrix& src1, const FloatingJntSpaceInertiaMatrix& src2)
    {return Equal(src1,src2);}
    //bool operator!=(const FloatingJntSpaceInertiaMatrix& src1,const FloatingJntSpaceInertiaMatrix& src2){return Equal(src1,src2);};

}

}
