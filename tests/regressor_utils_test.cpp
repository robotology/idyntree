#include <kdl_codyco/regressor_utils.hpp>

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
   
    
   return 0;
}
