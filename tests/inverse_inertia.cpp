#include <kdl_codyco/utils.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>

using namespace KDL;
using namespace KDL::CoDyCo;

int main()
{
	Twist t1 = 1*Twist(Vector(1,-2.21,3),Vector(-32,45.343,-0.45));
	Wrench m1;
	Twist t11;
	
	Wrench m2 = 1*Wrench(Vector(-23,0.22,1),Vector(-23,43.12,90.12));
	Twist t2;
	Wrench m22;
	
	RigidBodyInertia I = 100*RigidBodyInertia(23,Vector(-23,02.23,65),RotationalInertia(0.48424, 0.40606,1.56757,  0.23375 ,0.58465, 0.62535));
	
	t11 = (I*t1)/I;
	
	m22 = I*(m2/I);
	
	Twist error_twist = t1-t11;
	Wrench error_wrench = m2-m22; 
	
    /*
	std::cout << t1 << " " << t11 << std::endl;
	std::cout << error_twist << " ( " << (error_twist.vel.Norm() + error_twist.rot.Norm()) << " ) " << std::endl;
	std::cout << error_wrench << " ( " <<  (error_wrench.force.Norm() + error_wrench.torque.Norm())<< " ) " << std::endl;
	*/
    if( (error_twist.vel.Norm() + error_twist.rot.Norm()) > 1e-6 ) return -1;
    if( (error_wrench.force.Norm() + error_wrench.torque.Norm()) > 1e-6 ) return -1;
	return 0;
}
