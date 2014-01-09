
#include <iCub/iDynTree/iCubTree.h>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <kdl/frames.hpp>

#include <yarp/os/Random.h>
#include <yarp/os/Time.h>

#include <yarp/math/api.h>
#include <yarp/os/Log.h>

#include <iCub/ctrl/math.h>


using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace iCub::iDynTree;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;


using namespace std;

void set_random_vector(yarp::sig::Vector & vec, yarp::os::Random & rng, double coeff=1.0)
{
    for( int i=0; i < (int)vec.size(); i++ ) {
        vec[i] =  coeff*M_PI*rng.uniform();
    }
}

Matrix adjoint_twist(Matrix H)
{
    Matrix R = H.submatrix(0,2,0,2);
    Vector r = H.submatrix(0,2,0,3).getCol(3);
    //std::cout << "~~~~~~~ " << std::endl << " r " << r.toString() << std::endl;
    Matrix adj(6,6);
    adj.zero();
    adj.setSubmatrix(R,0,0);
    adj.setSubmatrix(R,3,3);
    adj.setSubmatrix(crossProductMatrix(r)*R,0,3);
    return adj;
}

Matrix SE3Inv(const Matrix &H)
{    
    if ((H.rows()!=4) || (H.cols()!=4))
    {

        return Matrix(0,0);
    }

    Vector p(4);
    p[0]=H(0,3);
    p[1]=H(1,3);
    p[2]=H(2,3);
    p[3]=1.0;

    Matrix invH=H.transposed();
    p=invH*p;
        
    invH(0,3)=-p[0];
    invH(1,3)=-p[1];
    invH(2,3)=-p[2];
    invH(3,0)=invH(3,1)=invH(3,2)=0.0;

    return invH;
}



void iDyn_print_velocity_acceleration(const yarp::sig::Vector & lin_vel, const yarp::sig::Vector & ang_vel, const yarp::sig::Vector & lin_acc, const yarp::sig::Vector & ang_acc, std::string link_name)
{
    Vector v(6), a(6);
    v.setSubvector(0,lin_vel);
    v.setSubvector(3,ang_vel);

    a.setSubvector(0,lin_acc);
    a.setSubvector(3,ang_acc);

    std::cout <<"Velocity of the " << link_name << endl
    << v.toString() << endl
    <<"Acceleration of the " << link_name << endl
    << a.toString() << endl;
    return;
}

void iDyn_compose_velocity_acceleration(const yarp::sig::Vector & lin_vel, const yarp::sig::Vector & ang_vel, const yarp::sig::Vector & lin_acc, const yarp::sig::Vector & ang_acc, Vector & v, Vector & a)
{
    v = Vector(6);
    a = Vector(6);
    v.setSubvector(0,lin_vel);
    v.setSubvector(3,ang_vel);

    a.setSubvector(0,lin_acc);
    a.setSubvector(3,ang_acc);
    return;
}


void iDynTree_print_velocity_acceleration(DynTree & icub_idyntree, const std::string link_name)
{    
    std::cout <<"Velocity of the " << link_name << endl;
    cout << icub_idyntree.getVel(icub_idyntree.getLinkIndex((link_name))).toString() << endl;
    cout <<"Acceleration of " << link_name << endl;
    cout << icub_idyntree.getAcc(icub_idyntree.getLinkIndex(link_name)).toString() << endl;
}

void set_random_q_dq_ddq(yarp::os::Random & rng, iCubTree & icub_tree)
{
    double pos_c = 0.0,vel_c = 0.0,acc_c =0.0;
    pos_c = 2.0;
    vel_c = 1.0;
    acc_c = 4.0;
    Vector q(icub_tree.getNrOfDOFs());            
    set_random_vector(q,rng,pos_c);
    icub_tree.setAng(q);
    
    Vector dq(icub_tree.getNrOfDOFs());            
    set_random_vector(dq,rng,vel_c);
    dq[1] = 1000.0;
    icub_tree.setDAng(dq);

    Vector ddq(icub_tree.getNrOfDOFs());            
    set_random_vector(ddq,rng,acc_c);
    icub_tree.setD2Ang(ddq);
    
    return;
}

KDL::Tree getSnake()
{
    KDL::Tree snake('fake_base_link');
    addSegment(Segment("snake_seg0",Joint("snake_jnt0",Joint::None),
                                   Frame::DH(2.5,M_PI_2,3,0.3),
                                   RigidBodyInertia(10,Vector(1,2,3),RotationalInertia(1,3,4))));
    addSegment(Segment("snake_seg1",Joint("snake_jnt1",Joint::RotZ),
                                   Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   RigidBodyInertia(2,Vector(1,-2,3),RotationalInertia(8,3,4))));
    addSegment(Segment("snake_seg2",Joint("snake_jnt2",Joint::RotZ),
                                   Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   RigidBodyInertia(2,Vector(1,-2,3),RotationalInertia(8,3,4))));
    addSegment(Segment("snake_seg3",Joint("snake_jnt3",Joint::RotZ),
                                   Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   RigidBodyInertia(2,Vector(1,-2,3),RotationalInertia(8,3,4))));
    addSegment(Segment("snake_seg4",Joint("snake_ft0",Joint::None),
                                   Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   RigidBodyInertia(2,Vector(1,-2,3),RotationalInertia(8,3,4))));
    addSegment(Segment("snake_seg5",Joint("snake_jnt5",Joint::RotZ),
                                   Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   RigidBodyInertia(2,Vector(1,-2,3),RotationalInertia(8,3,4))));
    addSegment(Segment("snake_seg6",Joint("snake_jnt6",Joint::RotZ),
                                   Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   RigidBodyInertia(2,Vector(1,-2,3),RotationalInertia(8,3,4))));
    addSegment(Segment("snake_seg7",Joint("snake_jnt7",Joint::RotZ),
                                   Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   RigidBodyInertia(2,Vector(1,-2,3),RotationalInertia(8,3,4))));
    addSegment(Segment("snake_seg8",Joint("snake_jnt8",Joint::RotZ),
                                   Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   RigidBodyInertia(2,Vector(1,-2,3),RotationalInertia(8,3,4))))
}

int main()
{
    
    //Initializing the random number generator
    yarp::os::Random rng;
    rng.seed(yarp::os::Time::now());
    
    //
    KDL::Tree snake = getSnake();
    
    //Creating the DynTree : kinematic/dynamics structure, force torque sensors, imu sensor
    iCub::iDynTree::DynTree snake_dyntree();
    
    snake_dyntree->setInertialMeasure();
    snake_dyntree->setSensorMeasurements();
    snake_dyntree->setAng();
    snake_dyntree->setDAng();
    snake_dyntree->sedD2Ang();
    snake_dyntree->setContacts();
    
    snake_dyntree->kinematicRNEA();
    snake_dyntree.estimateContactForces();
    snake_dyntree->dynamicRNEA();
    snake_dyntree.getContacts();
    
    //Check sums work out on the two subchains (if yes ok, if no FAIL)
    
}
