
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

    yarp::sig::Matrix H_w2b(4,4);
    H_w2b.eye();

    pos_c = 0.0;
    vel_c = 0.0;
    acc_c = 0.0;

    double world_orient = 1.0;
    double world_pos = 1.0;

    double base_vel_c = 1.0;
    double base_acc_c = 1.0;

    YARP_ASSERT(icub_tree.setWorldBasePose(H_w2b));

    std::cout << "iDynTree Jacobian test world pose " << icub_tree.getWorldBasePose().toString() << std::endl;


    Vector q(icub_tree.getNrOfDOFs());
    set_random_vector(q,rng,pos_c);
    icub_tree.setAng(q);

    Vector dq(icub_tree.getNrOfDOFs());
    set_random_vector(dq,rng,vel_c);
    //dq[1] = 1000.0;
    icub_tree.setDAng(dq);

    Vector ddq(icub_tree.getNrOfDOFs());
    set_random_vector(ddq,rng,acc_c);
    icub_tree.setD2Ang(ddq);

    Vector base_vel(6,0.0), base_acc(6,0.0);

    set_random_vector(base_vel,rng,base_vel_c);
    set_random_vector(base_acc,rng,base_acc_c);

    base_acc[3] = base_acc[4] = base_acc[5] = 0.0;

    icub_tree.setKinematicBaseVelAcc(base_vel,base_acc);

    //std::cout << "iDynTreeJacobianTest: Setting base_acc " << base_acc.toString() << " ( " << norm(base_acc) << " ) " <<  std::endl;

    YARP_ASSERT(icub_tree.kinematicRNEA());

    //std::cout << "iDynTreeJacobianTest: Acc at the base " << icub_tree.getAcc(icub_tree.getLinkIndex("root_link")).toString() << " ( " << norm(icub_tree.getAcc(icub_tree.getLinkIndex("root_link"))) << " ) " << std::endl;


    return;
}

int main()
{

    //To compare real com acceleration and the one calculated with the jacobian
    //We need to compute kinematicRNEA using root link as the kinematic source
    std::string kinematic_base_link_name = "root_link";

    double tol = 1e-5;

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
    icub_idyntree_version.feet_ft = true;

    //The iCubTree is istantiated
    //note that the serialization used is the one used in iDyn, while the
    //default one is the one used in skinDynLib
    int verbose = 1;
    iCubTree icub_idyntree(icub_idyntree_version,kinematic_base_link_name,IDYN_SERIALIZATION,verbose);

    //We fill the robot state with random values, for testing
    //in reality this should be filled with value read from the robot
    //NB: the serialization of q, dq, ddq is defined in the constructor of iCubTree
    set_random_q_dq_ddq(rng,icub_idyntree);

    yarp::sig::Matrix jac;

    //Test COM for leg
    icub_idyntree.getCOMJacobian(jac);
    icub_idyntree.getCOMJacobian(jac,"torso");
    icub_idyntree.getCOMJacobian(jac,"head");
    icub_idyntree.getCOMJacobian(jac,"left_arm");
    icub_idyntree.getCOMJacobian(jac,"right_arm");
    icub_idyntree.getCOMJacobian(jac,"left_leg");
    icub_idyntree.getCOMJacobian(jac,"right_leg");



    return 0;

}
