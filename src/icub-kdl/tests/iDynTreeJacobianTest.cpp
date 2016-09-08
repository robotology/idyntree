
#include "testModels.h"

#include <iCub/iDynTree/TorqueEstimationTree.h>

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

void set_random_q_dq_ddq(yarp::os::Random & rng, DynTree & icub_tree)
{
    double pos_c = 0.0,vel_c = 0.0,acc_c =0.0;

    yarp::sig::Matrix H_w2b(4,4);
    H_w2b.eye();

    pos_c = 1.0;
    vel_c = 1.0;
    acc_c = 1.0;

    double world_orient = 1.0;
    double world_pos = 1.0;

    double base_vel_c = 1.0;
    double base_acc_c = 1.0;

    KDL::Frame H_w2b_kdl;

    H_w2b_kdl.M = KDL::Rotation::EulerZYX(world_orient*rng.uniform(),world_orient*rng.uniform(),world_orient*rng.uniform());

    H_w2b_kdl.p[0] = world_pos*M_PI*rng.uniform();
    H_w2b_kdl.p[1] = world_pos*M_PI*rng.uniform();
    H_w2b_kdl.p[2] = world_pos*M_PI*rng.uniform();


    H_w2b_kdl.Make4x4(H_w2b.data());

    yAssert(icub_tree.setWorldBasePose(H_w2b));

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

    yAssert(icub_tree.kinematicRNEA());

    //std::cout << "iDynTreeJacobianTest: Acc at the base " << icub_tree.getAcc(icub_tree.getLinkIndex("root_link")).toString() << " ( " << norm(icub_tree.getAcc(icub_tree.getLinkIndex("root_link"))) << " ) " << std::endl;


    return;
}

int main(int argc, char** argv)
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


    //The iCubTree is istantiated
    //note that the serialization used is the one used in iDyn, while the
    //default one is the one used in skinDynLib
    std::string urdf_filename(IDYNTREE_TEST_MODELS_PATH"/icub.urdf");
    int verbose = 1;
    std::vector<std::string> sensors;
    DynTree icub_idyntree(urdf_filename,sensors,"imu_frame");

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
    v_rhand = icub_idyntree.getVel(r_hand_index);

    //By default the absloute jacobian is expressed in global reference frame
    //but it is possible to specify, via the local flag, to express them in local cordinates
    icub_idyntree.getJacobian(r_hand_index,abs_jacobian);

    v_rhand_abs_jac = abs_jacobian*icub_idyntree.getDQ_fb();

    //The relative jacobian is instead by default expressed in local coordinates
    //In this example we calculate the Jacobian between the two hands
    icub_idyntree.getRelativeJacobian(r_hand_index,l_hand_index,rel_jacobian);

    Vector v_rhand_local = icub_idyntree.getVel(r_hand_index,true);
    Vector v_lhand_local = icub_idyntree.getVel(l_hand_index,true);

    Matrix H_rhand_lhand = icub_idyntree.getPosition(r_hand_index,l_hand_index);
    Matrix R_rhand_lhand = H_rhand_lhand.submatrix(0,2,0,2);

    Vector v_lhand_wrt_rhand(6,0.0);

    v_lhand_wrt_rhand.setSubvector(0,R_rhand_lhand*v_lhand_local.subVector(0,2));
    v_lhand_wrt_rhand.setSubvector(3,R_rhand_lhand*v_lhand_local.subVector(3,5));

    cout << " v_lhand_wrt_rhand : " << v_lhand_wrt_rhand.toString() << std::endl;

    v_rhand_rel_jac = rel_jacobian*icub_idyntree.getDAng() + v_lhand_wrt_rhand;
    //v_rhand_rel_jac =  + adjoint_twist(icub_idyntree.getPosition(r_hand_index,l_hand_index))*icub_idyntree.getVel(l_hand_index,true);

    std::cout << "Comparison between velocities" << std::endl
              << "Real one          " << v_rhand.toString() << std::endl
              //<< "Relative jacobian " << v_rhand_rel_jac.toString() << std::endl
              << "Absolute jacobian " << v_rhand_abs_jac.toString() << std::endl
              //<< "Difference in norm " << norm(v_rhand-v_rhand_rel_jac) << std::endl
              << "Difference in norm " << norm(v_rhand-v_rhand_abs_jac) << std::endl
              << "Real local one      " << v_rhand_local.toString() << std::endl
              << "Relative jacobian local one      " << v_rhand_rel_jac.toString() << std::endl;





    //if( norm(v_rhand_local-v_rhand_rel_jac) > tol ) { return EXIT_FAILURE; }
    if( norm(v_rhand-v_rhand_abs_jac) > tol ) { return EXIT_FAILURE; }


    //For testing, it is also possible to check that the absolute velocity is computed correctly
    yarp::sig::Vector abs_v_rhand, abs_v_rhand_abs_jac;
    icub_idyntree.getJacobian(r_hand_index,abs_jacobian);
    abs_v_rhand = icub_idyntree.getVel(r_hand_index);
    abs_v_rhand_abs_jac = abs_jacobian*icub_idyntree.getDQ_fb();

    std::cout << "Comparison between velocities expressed in world frame" << std::endl
              << "Real one           " << abs_v_rhand.toString() << std::endl
              << "Absolute jacobian  " << abs_v_rhand_abs_jac.toString() << std::endl
              << "Difference in norm " << norm(abs_v_rhand-abs_v_rhand_abs_jac) << std::endl;

    if( norm(abs_v_rhand-abs_v_rhand_abs_jac) > tol ) { return EXIT_FAILURE; }


    //We can also add a testing on com DJdq (only on "linear" part for now)
    Matrix com_jacobian_6d, com_jacobian;
    icub_idyntree.getCOMJacobian(com_jacobian_6d);

    com_jacobian = com_jacobian_6d.submatrix(0,2,0,com_jacobian_6d.cols()-1);
    yarp::sig::Vector v_com, v_com_jacobian;
    v_com = icub_idyntree.getVelCOM();
    yAssert( com_jacobian.cols() == icub_idyntree.getNrOfDOFs()+6);
    v_com_jacobian = com_jacobian*icub_idyntree.getDQ_fb();

    std::cout << "Comparison between com velocities" << std::endl
              << "Real one     " << v_com.toString() << std::endl
              << "Jacobian one " << v_com_jacobian.toString() << std::endl;

    if( norm(v_com-v_com_jacobian) > tol ) { return EXIT_FAILURE; }



    iCub::iDynTree::DynTree waist_imu_icub(urdf_filename,sensors,"root_link");
    yarp::sig::Vector a_com, a_com_jacobian;
    a_com = icub_idyntree.getAccCOM();


    //World of waist_imu_icub is the same of icub_idyntree: root_link
    yarp::sig::Vector six_zeros(6,0.0);
    yarp::sig::Vector dof_zeros(icub_idyntree.getNrOfDOFs(),0.0);

    yAssert(waist_imu_icub.setWorldBasePose(icub_idyntree.getWorldBasePose()));
    waist_imu_icub.setKinematicBaseVelAcc(icub_idyntree.getVel(icub_idyntree.getLinkIndex(kinematic_base_link_name)),
                                          six_zeros);

    waist_imu_icub.setAng(icub_idyntree.getAng());
    waist_imu_icub.setDAng(icub_idyntree.getDAng());
    waist_imu_icub.setD2Ang(dof_zeros);
    yAssert(waist_imu_icub.kinematicRNEA());
    yAssert(icub_idyntree.kinematicRNEA());

    yAssert(icub_idyntree.getLinkIndex(kinematic_base_link_name) == waist_imu_icub.getLinkIndex(kinematic_base_link_name));

    std::cout << "Acc at the base " << icub_idyntree.getAcc(icub_idyntree.getLinkIndex(kinematic_base_link_name)).toString() << std::endl;
    a_com_jacobian = com_jacobian*icub_idyntree.getD2Q_fb() + waist_imu_icub.getAccCOM();



    std::cout << "Comparison between com accelerations" << std::endl
              << "Real one     " << a_com.toString() << std::endl
              << "Jacobian one " << a_com_jacobian.toString() << std::endl
              << "First part " << std::endl << (com_jacobian).submatrix(0,2,0,5).toString() << std::endl
              << "First part vec " << icub_idyntree.getD2Q_fb().toString() << std::endl
              << "Second part " << waist_imu_icub.getAccCOM().toString() << std::endl;

    if( norm(a_com-a_com_jacobian) > tol ) {
        std::cerr << "iDynTreeJacobianTest failed: Consistency error between com accelerations " << std::endl;
        return EXIT_FAILURE;
    }

    //Test the new getCentroidalMomentum
    yarp::sig::Vector centroidal_momentum = icub_idyntree.getCentroidalMomentum();

    yarp::sig::Matrix centroidal_momentum_jacobian;

    icub_idyntree.getCentroidalMomentumJacobian(centroidal_momentum_jacobian);

    yarp::sig::Vector centroidal_momentum_with_jac = centroidal_momentum_jacobian*icub_idyntree.getDQ_fb();

    std::cout << "Comparison between centroidal momentums" << std::endl
              << "Real one     " << centroidal_momentum.toString() << std::endl
              << "Jacobian one " << centroidal_momentum_with_jac.toString() << std::endl;


    if( norm(centroidal_momentum-centroidal_momentum_with_jac) > tol ) {
        std::cerr << "iDynTreeJacobianTest failed: Consistency error between centroidal momentums " << std::endl;
        return EXIT_FAILURE;
    }


    return 0;

}
