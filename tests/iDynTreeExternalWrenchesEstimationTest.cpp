
#include <iCub/iDynTree/iCubTree.h>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <kdl/frames.hpp>

#include <yarp/os/Random.h>
#include <yarp/os/Time.h>

#include <yarp/math/api.h>
#include <yarp/os/Log.h>

#include <iCub/ctrl/math.h>
#include <iCub/skinDynLib/dynContactList.h>


using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace iCub::iDynTree;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::skinDynLib;

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

void set_random_IMU_q_dq_ddq(yarp::os::Random & rng, DynTree & icub_tree)
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
    
    Vector imu_ang_vel(3), imu_ang_acc(3), imu_lin_acc(3);
    set_random_vector(imu_ang_vel,rng,vel_c);
    set_random_vector(imu_ang_acc,rng,acc_c);
    set_random_vector(imu_lin_acc,rng,acc_c);
    
    icub_tree.setInertialMeasure(imu_ang_vel,imu_ang_acc,imu_lin_acc);
    
    return;
}

KDL::Tree getSnake()
{
    KDL::Tree snake("fake_base_link");
    KDL::Chain snake_chain;
    snake_chain.addSegment(KDL::Segment("snake_seg0",KDL::Joint("snake_jnt0",KDL::Joint::None),
                                   KDL::Frame::DH(2.5,M_PI_2,3,0.3),
                                   KDL::RigidBodyInertia(10,KDL::Vector(1,2,3),KDL::RotationalInertia(1,3,4))));
    snake_chain.addSegment(KDL::Segment("snake_seg1",KDL::Joint("snake_jnt1",KDL::Joint::RotZ),
                                   KDL::Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   KDL::RigidBodyInertia(2,KDL::Vector(1,-2,3),KDL::RotationalInertia(8,3,4))));
    snake_chain.addSegment(KDL::Segment("snake_seg2",KDL::Joint("snake_jnt2",KDL::Joint::RotZ),
                                   KDL::Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   KDL::RigidBodyInertia(2,KDL::Vector(1,-2,3),KDL::RotationalInertia(8,3,4))));
    snake_chain.addSegment(KDL::Segment("snake_seg3",KDL::Joint("snake_jnt3",KDL::Joint::RotZ),
                                   KDL::Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   KDL::RigidBodyInertia(2,KDL::Vector(1,-2,3),KDL::RotationalInertia(8,3,4))));
    snake_chain.addSegment(KDL::Segment("snake_seg4",KDL::Joint("snake_ft0",KDL::Joint::None),
                                   KDL::Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   KDL::RigidBodyInertia(2,KDL::Vector(1,-2,3),KDL::RotationalInertia(8,3,4))));
    snake_chain.addSegment(KDL::Segment("snake_seg5",KDL::Joint("snake_jnt5",KDL::Joint::RotZ),
                                   KDL::Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   KDL::RigidBodyInertia(2,KDL::Vector(1,-2,3),KDL::RotationalInertia(8,3,4))));
    snake_chain.addSegment(KDL::Segment("snake_seg6",KDL::Joint("snake_jnt6",KDL::Joint::RotZ),
                                   KDL::Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   KDL::RigidBodyInertia(2,KDL::Vector(1,-2,3),KDL::RotationalInertia(8,3,4))));
    snake_chain.addSegment(KDL::Segment("snake_seg7",KDL::Joint("snake_jnt7",KDL::Joint::RotZ),
                                   KDL::Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   KDL::RigidBodyInertia(2,KDL::Vector(1,-2,3),KDL::RotationalInertia(8,3,4))));
    snake_chain.addSegment(KDL::Segment("snake_seg8",KDL::Joint("snake_jnt8",KDL::Joint::RotZ),
                                   KDL::Frame::DH(0.5,M_PI_2/4,-3,2.3),
                                   KDL::RigidBodyInertia(2,KDL::Vector(1,-2,3),KDL::RotationalInertia(8,3,4))));
    
    snake.addChain(snake_chain,"fake_base_link");
    
    return snake;
}

int main()
{
    
    //Initializing the random number generator
    yarp::os::Random rng;
    rng.seed(yarp::os::Time::now());
    
    //Creating the DynTree : kinematic/dynamics structure, force torque sensors, imu sensor
    KDL::Tree snake = getSnake();
    std::vector<std::string> ft_sensors;
    ft_sensors.push_back("snake_ft0");
    std::string imu_link = "snake_seg2";
    iCub::iDynTree::DynTree snake_dyntree(snake,ft_sensors,imu_link);
    
    //Assign random kinematic state
    set_random_IMU_q_dq_ddq(rng,snake_dyntree);
    
    //Assign random sensor_reading 
    Vector ft_sensor(6);
    set_random_vector(ft_sensor,rng,10);
    
    snake_dyntree.setSensorMeasurement(0,ft_sensor);

    //Set only two unknown wrenches, one for subchain, so the estimation problem is well posed
    iCub::skinDynLib::BodyPart DEFAULT_BODY_PART_ID = (iCub::skinDynLib::BodyPart)0;
    
    dynContactList input_contact_list, output_contact_list;
    const int first_contact_link = 1;
    const int second_contact_link = 7;
    dynContact first_contact(DEFAULT_BODY_PART_ID,first_contact_link,yarp::sig::Vector(3,0.0));
    dynContact second_contact(DEFAULT_BODY_PART_ID,second_contact_link,yarp::sig::Vector(3,0.0));
    
    input_contact_list.push_back(first_contact);
    input_contact_list.push_back(second_contact);
    
    std::cout << "Snake_dyntree partition: " << snake_dyntree.getKDLUndirectedTree().getPartition().toString() << std::endl;
    
    snake_dyntree.setContacts(input_contact_list);
    
    snake_dyntree.kinematicRNEA();
    snake_dyntree.estimateContactForces();
    snake_dyntree.dynamicRNEA();
    
    output_contact_list = snake_dyntree.getContacts();
    
    if( output_contact_list.size() != 2 ) { std::cout << "Error: Estimated more than two contacts."; return EXIT_FAILURE; }
    
    dynContact first_contact_estimated = output_contact_list[0];
    dynContact second_contact_estimated = output_contact_list[1];

    //Get internal subtree dynamics
    std::vector<yarp::sig::Vector> int_dyn = snake_dyntree.getSubTreeInternalDynamics();
    
    //All the forces should be have an equilibrium on each subgraph
    yarp::sig::Vector ext_f_1 = snake_dyntree.getPosition(first_contact_link).submatrix(0,2,0,2)*first_contact_estimated.getForce();
    yarp::sig::Vector ext_f_2 = snake_dyntree.getPosition(second_contact_link).submatrix(0,2,0,2)*second_contact_estimated.getForce();
    
    yarp::sig::Vector int_f_1 = int_dyn[0].subVector(0,2);
    yarp::sig::Vector int_f_2 = int_dyn[1].subVector(0,2);
    
    yarp::sig::Vector ext_t_1 = snake_dyntree.getPosition(first_contact_link).submatrix(0,2,0,2)*first_contact_estimated.getMoment();
    yarp::sig::Vector ext_t_2 = snake_dyntree.getPosition(second_contact_link).submatrix(0,2,0,2)*second_contact_estimated.getMoment();
    
    yarp::sig::Vector int_w_1 = int_dyn[0];
    yarp::sig::Vector int_w_2 = int_dyn[1];
    
    yarp::sig::Vector ext_w_1 = cat(ext_f_1,ext_t_1);
    yarp::sig::Vector ext_w_2 = cat(ext_f_2,ext_t_2);
    
    
    yarp::sig::Vector sens_f_1 = ft_sensor.subVector(0,2);
    yarp::sig::Vector sens_f_2 = -1*sens_f_1;
    
    yarp::sig::Vector sens_w_1 = ft_sensor;
    yarp::sig::Vector sens_w_2 = -1*sens_w_1;
    
    
    //
    std::cout << "ext_f_1 norm: " << yarp::math::norm(ext_f_1) << std::endl;
    std::cout << "ext_f_2 norm: " << yarp::math::norm(ext_f_2) << std::endl;
    std::cout << "int_f_1 norm: " << yarp::math::norm(int_f_1) << std::endl;
    std::cout << "int_f_2 norm: " << yarp::math::norm(int_f_2) << std::endl;
    std::cout << "sens_f_1 norm: " << yarp::math::norm(sens_f_1) << std::endl;
    std::cout << "sens_f_2 norm: " << yarp::math::norm(sens_f_2) << std::endl;
    std::cout << "ext_f_1 " << ext_f_1.size() << " " << int_f_1.size() << std::endl;
    std::cout << "ext_f_1-int_f_1 norm: " << yarp::math::norm(ext_f_1-int_f_1) << std::endl;
    std::cout << "ext_f_2-int_f_2 norm: " << yarp::math::norm(ext_f_2-int_f_2) << std::endl;
    
    double tol = 1e-6;
    
    if( fabs(yarp::math::norm(ext_f_1-int_f_1)- yarp::math::norm(sens_f_1)) > tol ) { return EXIT_FAILURE; }
    if( fabs(yarp::math::norm(ext_f_2-int_f_2)- yarp::math::norm(sens_f_2)) > tol ) { return EXIT_FAILURE; }

    /*
    std::cout << "ext_w_1 norm: " << yarp::math::norm(ext_w_1) << std::endl;
    std::cout << "ext_w_2 norm: " << yarp::math::norm(ext_w_2) << std::endl;
    std::cout << "int_w_1 norm: " << yarp::math::norm(int_w_1) << std::endl;
    std::cout << "int_w_2 norm: " << yarp::math::norm(int_w_2) << std::endl;
    std::cout << "sens_w_1 norm: " << yarp::math::norm(sens_w_1) << std::endl;
    std::cout << "sens_w_2 norm: " << yarp::math::norm(sens_w_2) << std::endl;
    std::cout << "ext_w_1 " << ext_w_1.size() << " " << int_w_1.size() << std::endl;
    std::cout << "ext_w_1-int_w_1 norm: " << yarp::math::norm(ext_w_1-int_w_1) << std::endl;
    std::cout << "ext_w_2-int_w_2 norm: " << yarp::math::norm(ext_w_2-int_w_2) << std::endl;
    */
    
    return EXIT_SUCCESS;
}
