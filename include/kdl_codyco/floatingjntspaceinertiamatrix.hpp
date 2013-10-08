// Copyright  (C)  2013  Silvio Traversaro <silvio dot traversaro at iit dot it>
// Copyright  (C)  2009  Dominick Vanthienen <dominick dot vanthienen at mech dot kuleuven dot be>

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

#ifndef KDL_CODYCO_FLOATINGJNTSPACEINERTIAMATRIX_HPP
#define KDL_CODYCO_FLOATINGJNTSPACEINERTIAMATRIX_HPP

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <Eigen/Core>

namespace KDL
{
namespace CoDyCo 
{
    /**
     * @brief This class represents an fixed size matrix containing
     * the Floating Base Joint-Space Inertia Matrix of a KDL::Tree.
     *
     * \warning An object constructed with the default constructor provides
     * a valid, but inert, object. Many of the member functions will do
     * the correct thing and have no affect on this object, but some 
     * member functions can _NOT_ deal with an inert/empty object. These 
     * functions will assert() and exit the program instead.
     */	

    class FloatingJntSpaceInertiaMatrix
    {
    public:
        Eigen::MatrixXd data;

        /** Construct with _no_ data array
         * @post NULL == data
         * @post 0 == rows()
         * @warning use of an object constructed like this, without
         * a resize() first, may result in program exit! See class
         * documentation.
         */
        FloatingJntSpaceInertiaMatrix();
        /**
         * Constructor of the Floating Base Joint-Space Inertia Matrix
         *
         * @param size of the matrix, this cannot be changed
         * afterwards. Size rows and size columns. As this is a
         * Floating Base Inertia Matrix, for a typical robot this will be 6+getNrOfDOFs()
         * @pre 0 < size
         * @post NULL != data
         * @post 0 < rows()
         * @post all elements in data have 0 value
         */
        explicit FloatingJntSpaceInertiaMatrix(int size);
        /** Copy constructor 
         * @note Will correctly copy an empty object
         */
        FloatingJntSpaceInertiaMatrix(const FloatingJntSpaceInertiaMatrix& arg);
        ~FloatingJntSpaceInertiaMatrix();
        
        /** Resize the array 
         * @warning This causes a dynamic allocation (and potentially 	
         * also a dynamic deallocation). This _will_ negatively affect
         * real-time performance! 
         *
         * @post newSize == rows()
         * @post NULL != data
         * @post all elements in data have 0 value
         */
        void resize(unsigned int newSize);
		
        FloatingJntSpaceInertiaMatrix& operator = ( const FloatingJntSpaceInertiaMatrix& arg);
        /**
         * get_item operator for the joint matrix
         *
         *
         * @return the joint value at position i, starting from 0
         * @pre 0 != size (ie non-default constructor or resize() called)
         */
        double operator()(unsigned int i,unsigned int j)const;
        /**
         * set_item operator
         *
         * @return reference to the joint value at position i,starting
         *from zero.
         * @pre 0 != size (ie non-default constructor or resize() called)
         */
        double& operator()(unsigned int i,unsigned int j);
        /**
         * Returns the number of rows and columns of the matrix
         *
         */
        unsigned int rows()const;
        /**
         * Returns the number of columns of the matrix.
         */
        unsigned int columns()const;

        /**
         * Function to add two joint matrix, all the arguments must
         * have the same size: A + B = C. This function is
         * aliasing-safe, A or B can be the same array as C.
         *
         * @param src1 A
         * @param src2 B
         * @param dest C
         */
        friend void Add(const FloatingJntSpaceInertiaMatrix& src1,const FloatingJntSpaceInertiaMatrix& src2,FloatingJntSpaceInertiaMatrix& dest);
        /**
         * Function to subtract two joint matrix, all the arguments must
         * have the same size: A - B = C. This function is
         * aliasing-safe, A or B can be the same array as C.
         *
         * @param src1 A
         * @param src2 B
         * @param dest C
         */
        friend void Subtract(const FloatingJntSpaceInertiaMatrix& src1,const FloatingJntSpaceInertiaMatrix& src2,FloatingJntSpaceInertiaMatrix& dest);
        /**
         * Function to multiply all the array values with a scalar
         * factor: A*b=C. This function is aliasing-safe, A can be the
         * same array as C.
         *
         * @param src A
         * @param factor b
         * @param dest C
         */
        friend void Multiply(const FloatingJntSpaceInertiaMatrix& src,const double& factor,FloatingJntSpaceInertiaMatrix& dest);
        /**
         * Function to divide all the array values with a scalar
         * factor: A/b=C. This function is aliasing-safe, A can be the
         * same array as C.
         *
         * @param src A
         * @param factor b
         * @param dest C
         */
        friend void Divide(const FloatingJntSpaceInertiaMatrix& src,const double& factor,FloatingJntSpaceInertiaMatrix& dest);
        
        /**
         * 
         */
        friend void Multiply(const FloatingJntSpaceInertiaMatrix& src, const Twist & base_vel, const JntArray& ddq, Wrench & base_wrench, JntArray& tau);
        
        /**
         * Function to set all the values of the array to 0
         *
         * @param array
         */
        friend void SetToZero(FloatingJntSpaceInertiaMatrix& matrix);
        /**
         * Function to check if two matrices are the same with a
         *precision of eps
         *
         * @param src1
         * @param src2
         * @param eps default: epsilon
         * @return true if each element of src1 is within eps of the same
		 * element in src2, or if both src1 and src2 have no data (ie 0==rows())
         */
        friend bool Equal(const FloatingJntSpaceInertiaMatrix& src1,const FloatingJntSpaceInertiaMatrix& src2,double eps=epsilon);

        friend bool operator==(const FloatingJntSpaceInertiaMatrix& src1,const FloatingJntSpaceInertiaMatrix& src2);
        //friend bool operator!=(const FloatingJntSpaceInertiaMatrix& src1,const FloatingJntSpaceInertiaMatrix& src2);
        };

    bool operator==(const FloatingJntSpaceInertiaMatrix& src1,const FloatingJntSpaceInertiaMatrix& src2);
    //bool operator!=(const FloatingJntSpaceInertiaMatrix& src1,const FloatingJntSpaceInertiaMatrix& src2);

}
}

#endif
