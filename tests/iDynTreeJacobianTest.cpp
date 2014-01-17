
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

int main()
{
    
    //Initializing the random number generator
    yarp::os::Random rng;
    rng.seed(yarp::os::Time::now());
    
    ////////////////////////////////////////////////////////////////////
    //// iCubTree 
    ////////////////////////////////////////////////////////////////////    
    //Similarly in iDynTree a iCubTree_version_tag structure is defined
    iCubTree_version_tag icub_idyntree_version;
    
    icub_idyntree_version.head_version = 2;
    icub_idyntree_version.legs_version = 2;
    
    //The iCubTree is istantiated
    //note that the serialization used is the one used in iDyn, while the 
    //default one is the one used in skinDynLib
    iCubTree icub_idyntree(icub_idyntree_version,IDYN_SERIALIZATION);

    Vector w0(3,0.0);
    Vector dw0(3,0.0);
    Vector ddp0(3,0.0);
    ddp0[2] = 9.8;
    
    //The setInertialMeasure impose the velocity in the imu, like in iCubWholeBody
    icub_idyntree.setInertialMeasure(w0,dw0,ddp0);
    
    //We fill the robot state with random values, for testing
    //in reality this should be filled with value read from the robot 
    //NB: the serialization of q, dq, ddq is defined in the constructor of iCubTree
    set_random_q_dq_ddq(rng,icub_idyntree);
       
    //std::cout << "Joint position" << std::endl;
    //std::cout << icub_idyntree.getAng().toString() << std::endl;
    
    
    //We then perform kinematic propagation
    icub_idyntree.kinematicRNEA();
       
    
    ////////////////////////////////////////////////////////////////////
    // Checking JACOBIANS
    ////////////////////////////////////////////////////////////////////
    
    //In iDynTree there are two type of Jacobians:
    //* The absolute jacobian, that has 6+NrOfDOFs columns and maps 
    //  the velocity of the floating base and the velocity of the joints to 
    //  the velocity of a link (check getJacobian documentation ) 
    //
    //* The relative jacobian, that has NrOfDOFs columsn and maps the 
    //  the velocity of the joints to the difference of the velocity of 
    //  two links (check getRelativeJacobian documentation)
    
    Matrix rel_jacobian;
    Matrix abs_jacobian;
    
    //We can calculate then the velocity of the right hand using the 
    //kinematic propagation or using the jacobians
    Vector v_rhand;
    Vector v_rhand_rel_jac;
    Vector v_rhand_abs_jac;
    
    //We get the index for the right hand and the left hand
    int r_hand_index = icub_idyntree.getLinkIndex("r_gripper");
    int l_hand_index = icub_idyntree.getLinkIndex("l_gripper");
    
    //By default the returned velocity is expressed in global reference frame (but with local reference point)
    //but it is possible to specify, via the local flag, to express them in local cordinates
    v_rhand = icub_idyntree.getVel(r_hand_index,true);
    
    //By default the absloute jacobian is expressed in global reference frame
    //but it is possible to specify, via the local flag, to express them in local cordinates
    icub_idyntree.getJacobian(r_hand_index,abs_jacobian,true);

    v_rhand_abs_jac = abs_jacobian*icub_idyntree.getDQ_fb();
    
    //The relative jacobian is instead by default expressed in local coordinates
    //In this example we calculate the Jacobian between the two hands
    icub_idyntree.getRelativeJacobian(r_hand_index,l_hand_index,rel_jacobian);
    
    //v_rhand_rel_jac = rel_jacobian*icub_idyntree.getDAng() + adjoint_twist(icub_idyntree.getPosition(r_hand_index,l_hand_index))*icub_idyntree.getVel(l_hand_index);
    
    std::cout << "Comparison between velocities" << std::endl 
              << "Real one          " << v_rhand.toString() << std::endl
              << "Relative jacobian " << v_rhand_rel_jac.toString() << std::endl
              << "Absolute jacobian " << v_rhand_abs_jac.toString() << std::endl;
             
    //For testing, it is also possible to check that the absolute velocity is computed correctly
    yarp::sig::Vector abs_v_rhand, abs_v_rhand_abs_jac;
    icub_idyntree.getJacobian(r_hand_index,abs_jacobian);
    abs_v_rhand = icub_idyntree.getVel(r_hand_index);
    abs_v_rhand_abs_jac = abs_jacobian*icub_idyntree.getDQ_fb();
         
    std::cout << "Comparison between velocities expressed in world frame" << std::endl 
              << "Real one          " << abs_v_rhand.toString() << std::endl
              << "Absolute jacobian " << abs_v_rhand_abs_jac.toString() << std::endl;
             
             
    
    return 0;
    
}
