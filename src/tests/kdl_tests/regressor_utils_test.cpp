/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <kdl_codyco/regressor_utils.hpp>

#include <kdl/frames_io.hpp>
#include <iostream>

using namespace KDL;
using namespace KDL::CoDyCo;

double random_double()
{
    return ((double)rand()-(RAND_MAX/2))/((double)RAND_MAX);
}

typedef Eigen::Matrix<double,6,1> Vector6d;

int main()
{
    double tol = 1e-10;

   //Testing WrenchTransformationMatrix
   Wrench random_wrench(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));
   KDL::Frame random_frame(Frame::DH(random_double(),random_double(),random_double(),random_double()));
   Wrench random_result = random_frame*random_wrench;
   Vector6d random_result_eigen1,random_result_eigen2;

   random_result_eigen1 = toEigen(random_result);
   random_result_eigen2 = WrenchTransformationMatrix(random_frame)*toEigen(random_wrench);

   //std::cout << random_result_eigen1 << std::endl;
   //std::cout << random_result_eigen2 << std::endl;
   //std::cout << random_result_eigen1-random_result_eigen2 << std::endl;

   if( (random_result_eigen1-random_result_eigen2).norm() > 1e-12 ) { return -1; }

   //Testing toKDLWrench
   Vector6d random_wrench_converted = toEigen(random_wrench);
   Wrench random_wrench_double_conversion = toKDLWrench(random_wrench_converted);

   std::cout << "Random wrench: " << std::endl;
   std::cout << random_wrench << std::endl;
   std::cout << random_wrench_converted << std::endl;
   std::cout << random_wrench_double_conversion << std::endl;


   for(int i=0; i < 6; i++ )
   {
       if( fabs(random_wrench_converted(i) - random_wrench(i)) > tol )
       {
           return -1;
       }

       if( fabs(random_wrench_double_conversion(i) - random_wrench(i)) > tol )
       {
           return -1;
       }
   }

   //Testing toKDLTwist
   Twist random_twist(Vector(random_double(),random_double(),random_double()),Vector(random_double(),random_double(),random_double()));
   Vector6d random_twist_converted = toEigen(random_twist);
   Twist random_twist_double_conversion = toKDLTwist(random_twist_converted);


   std::cout << "Random twist: " << std::endl;
   std::cout << random_twist << std::endl;
   std::cout << random_twist_converted << std::endl;
   std::cout << random_twist_double_conversion << std::endl;


   for(int i=0; i < 6; i++ )
   {
       if( fabs(random_twist_converted(i) - random_twist(i)) > tol )
       {
           return -1;
       }

       if( fabs(random_twist_double_conversion(i) - random_twist(i)) > tol )
       {
           return -1;
       }
   }


   return 0;
}
