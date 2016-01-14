#define _USE_MATH_DEFINES

#include "testModels.h"

#include <cmath>
#include <iCub/iDynTree/TorqueEstimationTree.h>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <kdl/frames.hpp>

#include <yarp/os/Random.h>
#include <yarp/os/Time.h>

#include <yarp/math/api.h>
#include <yarp/os/Log.h>

#include <iCub/ctrl/math.h>
#include <iCub/skinDynLib/dynContactList.h>
#include <kdl_codyco/regressors/dirl_utils.hpp>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/SubModel.h>
#include <iDynTree/Model/ModelTransformers.h>
#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/ContactWrench.h>
#include <iDynTree/Model/Dynamics.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/ModelIO/URDFModelImport.h>
#include <iDynTree/Estimation/ExternalWrenchesEstimation.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/iCub/skinDynLibConversions.h>

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

void set_random_IMU_q_dq_ddq(yarp::os::Random & rng, DynTree & icub_tree,
                             iDynTree::Vector3 & base_classicalProperAcc, iDynTree::Vector3 & base_angularVel, iDynTree::Vector3 & base_angularAcc,
                             iDynTree::JointPosDoubleArray& jointPos, iDynTree::JointDOFsDoubleArray& jointVel, iDynTree::JointDOFsDoubleArray& jointAcc,
                             double pos_c = 1.0, double vel_c = 1.0, double acc_c = 1.0
)
{
    Vector q(icub_tree.getNrOfDOFs());
    set_random_vector(q,rng,pos_c);
    q = icub_tree.setAng(q);
    iDynTree::toiDynTree(q,jointPos);

    Vector dq(icub_tree.getNrOfDOFs());
    set_random_vector(dq,rng,vel_c);
    dq[1] = 1000.0;
    dq = icub_tree.setDAng(dq);
    iDynTree::toiDynTree(dq,jointVel);


    Vector ddq(icub_tree.getNrOfDOFs());
    set_random_vector(ddq,rng,acc_c);
    ddq = icub_tree.setD2Ang(ddq);
    iDynTree::toiDynTree(ddq,jointAcc);


    Vector imu_ang_vel(3), imu_ang_acc(3), imu_lin_acc(3);
    set_random_vector(imu_ang_vel,rng,vel_c);
    set_random_vector(imu_ang_acc,rng,acc_c);
    set_random_vector(imu_lin_acc,rng,acc_c);
    iDynTree::toiDynTree(imu_ang_vel,base_angularVel);
    iDynTree::toiDynTree(imu_ang_acc,base_angularAcc);
    iDynTree::toiDynTree(imu_lin_acc,base_classicalProperAcc);

    icub_tree.setInertialMeasure(imu_ang_vel,imu_ang_acc,imu_lin_acc);

    return;
}

std::vector<iDynTree::FTSensorData> get_default_ft_sensors(std::vector<std::string> ft_serialization)
{
    std::vector<iDynTree::FTSensorData> ret;
    for(size_t i =0; i < ft_serialization.size(); i++ )
    {
        iDynTree::FTSensorData dat;
        dat.reference_joint = ft_serialization[i];
        dat.sensor_name = dat.reference_joint+"_sensor";
        dat.measure_direction = iDynTree::FTSensorData::CHILD_TO_PARENT;
        dat.frame = iDynTree::FTSensorData::CHILD_LINK_FRAME;
        dat.sensor_pose = KDL::Frame::Identity();

        ret.push_back(dat);
    }

    return ret;
}

bool addDoubleContact(iCub::iDynTree::TorqueEstimationTree & estimation_model,
                      const iDynTree::Model & model,
                      iDynTree::skinDynLibConversionsHelper & icub_skin_helper,
                      dynContactList & input_contact_list,
                      const std::string contactLink, const std::string contactFrame)
{
    int contactLinkIndex =  estimation_model.getLinkIndex(contactLink);
    estimation_model.addSkinDynLibAlias(contactLink,contactFrame,0,contactLinkIndex);
    icub_skin_helper.addSkinDynLibAlias(model,contactLink,contactFrame,0,contactLinkIndex);

    yarp::sig::Vector oneContactPoint(3);
    oneContactPoint.zero();
    oneContactPoint[0] = 1.0;
    dynContact contactOne((iCub::skinDynLib::BodyPart) 0, contactLinkIndex,oneContactPoint);
    input_contact_list.push_back(contactOne);

    yarp::sig::Vector twoContactPoint(3);
    twoContactPoint.zero();
    twoContactPoint[1] = 1.0;
    dynContact contactTwo((iCub::skinDynLib::BodyPart) 0, contactLinkIndex,twoContactPoint);
    input_contact_list.push_back(contactTwo);

    return true;
}

bool addSingleContact(iCub::iDynTree::TorqueEstimationTree & estimation_model,
                      const iDynTree::Model & model,
                      iDynTree::skinDynLibConversionsHelper & icub_skin_helper,
                      dynContactList & input_contact_list,
                      const std::string contactLink, const std::string contactFrame)
{
    int contactLinkIndex =  estimation_model.getLinkIndex(contactLink);
    estimation_model.addSkinDynLibAlias(contactLink,contactFrame,0,contactLinkIndex);
    icub_skin_helper.addSkinDynLibAlias(model,contactLink,contactFrame,0,contactLinkIndex);
    yarp::sig::Vector oneContactPoint(3);
    oneContactPoint.zero();
    oneContactPoint[0] = 1.0;
    dynContact contactOne((iCub::skinDynLib::BodyPart) 0, contactLinkIndex,oneContactPoint);
    input_contact_list.push_back(contactOne);

    return true;
}

void getJointSerializationFromDynTree(DynTree & dynTree,
                                      iDynTree::Model & model,
                                      std::vector<std::string>& joints)
{
    joints.resize(0);
    size_t nrOfJoints = dynTree.getNrOfLinks()-1;
    for(size_t j=0; j < nrOfJoints; j++ )
    {
        std::string jointName;
        bool ok = dynTree.getJunctionName(j,jointName);
        assert(ok);

        // if the joints is also in the model, add it
        if( model.getJointIndex(jointName) != iDynTree::JOINT_INVALID_INDEX )
        {
            joints.push_back(jointName);
        }
    }
}

int main(int argc, char ** argv)
{
    //Initializing the random number generator
    yarp::os::Random rng;
    rng.seed(0.0);

    std::vector<std::string> ft_serialization;
    ft_serialization.push_back("l_arm_ft_sensor");
    ft_serialization.push_back("r_arm_ft_sensor");
    ft_serialization.push_back("l_leg_ft_sensor");
    ft_serialization.push_back("l_foot_ft_sensor");
    ft_serialization.push_back("r_leg_ft_sensor");
    ft_serialization.push_back("r_foot_ft_sensor");

    std::vector<std::string> dof_serialization = std::vector<std::string>(0);

    //Creating the DynTree : kinematic/dynamics structure, force torque sensors, imu sensor
    std::string urdf_filename = getAbsModelPath("icub_skin_frames.urdf");
    iCub::iDynTree::TorqueEstimationTree * icub_model_estimation =
        new iCub::iDynTree::TorqueEstimationTree(urdf_filename,dof_serialization,ft_serialization);

    // Creating the new iDynTree based data structures
    iDynTree::Model fullModel, model;
    iDynTree::modelFromURDF(urdf_filename,fullModel);
    // Get the same serialization of joints used in iDynTree
    std::vector<std::string> usedJoints;
    getJointSerializationFromDynTree(*icub_model_estimation,fullModel,usedJoints);
    bool ok = iDynTree::createReducedModel(fullModel,usedJoints,model);
    if( !ok )
    {
        std::cerr << "Error in creating reduced model" << std::endl;
        return EXIT_FAILURE;
    }

    std::cerr << " old model number of dofs : " << icub_model_estimation->getNrOfDOFs() << std::endl;
    std::cerr << model.toString() << std::endl;

    assert(model.getNrOfDOFs() == icub_model_estimation->getNrOfDOFs());

    iDynTree::Traversal traversal;
    model.computeFullTreeTraversal(traversal);
    // Create sensors list with FT sensors (TODO)
    iDynTree::SensorsList sensors;
    //iDynTree::sensorsFromURDF(urdf_filename,model,sensors);

    // Create submodel decomposition
    std::cerr << model.toString() << std::endl;

    iDynTree::SubModelDecomposition subModels;
    ok = subModels.splitModelAlongJoints(model,traversal,ft_serialization);
    if( !ok )
    {
        std::cerr << "Error in splitting the model in submodels" << std::endl;
        return EXIT_FAILURE;
    }

    iDynTree::skinDynLibConversionsHelper iCubSkinHelper;

    // Base quantities
    iDynTree::Vector3 base_classicalProperAcc,base_angularVel,base_angularAcc;

    // position/velocity/acceleration
    iDynTree::JointPosDoubleArray jointPos(model);
    iDynTree::JointDOFsDoubleArray jointVel(model);
    iDynTree::JointDOFsDoubleArray jointAcc(model);

    // Data structures to encode the external forces
    iDynTree::LinkNetExternalWrenches netExternalWrenches(model);
    iDynTree::LinkContactWrenches     externalWrenches(model);

    // Data structure to encode the joint torques
    iDynTree::FreeFloatingGeneralizedTorques trqs(model);

    // Extend the freefloating state classes to easily set
    // the velocity/acceleration/force of the base using the different conventions

    //Assign random kinematic state
    set_random_IMU_q_dq_ddq(rng,*icub_model_estimation,
                            base_classicalProperAcc, base_angularVel,base_angularAcc,
                            jointPos,jointVel,jointAcc);

    //Assign random sensor_readings
    Vector ft_sensor(6);

    iDynTree::SensorsMeasurements sensMeas(sensors);

    for(int i = 0; i < icub_model_estimation->getNrOfFTSensors(); i++ )
    {
        set_random_vector(ft_sensor,rng,10);
        icub_model_estimation->setSensorMeasurement(i,ft_sensor);
        iDynTree::Wrench ft_sensor_idyntree;
        iDynTree::toiDynTree(ft_sensor,ft_sensor_idyntree);
        assert(sensors.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) > i);

    }

    dynContactList input_contact_list, output_contact_list;

    // Populate input list and mapping between skinDynLib links and
    // iDynTree links
    addSingleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "chest","chest_skin_frame");
    addSingleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "l_hand","l_hand_dh_frame");
    addSingleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "r_hand","r_hand_dh_frame");
    addSingleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "r_foot","r_foot_dh_frame");
    addSingleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "l_foot","l_foot_dh_frame");
    addSingleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "l_lower_leg","l_lower_leg");
    addSingleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "r_lower_leg","r_lower_leg");

    addDoubleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "chest","chest_skin_frame");
    addDoubleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "l_hand","l_hand_dh_frame");
    addDoubleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "r_hand","r_hand_dh_frame");
    addDoubleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "r_foot","r_foot_dh_frame");
    addDoubleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "l_foot","l_foot_dh_frame");
    addDoubleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "l_lower_leg","l_lower_leg");
    addDoubleContact(*icub_model_estimation,model,iCubSkinHelper,input_contact_list,
                     "r_lower_leg","r_lower_leg");

    // Create iDynTree-compatible input list
    iDynTree::LinkUnknownWrenchContacts unknownContacts(model);

    iCubSkinHelper.fromSkinDynLibToiDynTree(model,input_contact_list,unknownContacts);
    std::cerr << "skinDynLib unknownContacts : " << std::endl;
    std::cerr << input_contact_list.toString() << std::endl;

    std::cerr << "iDynTree unknownContacts : " << std::endl;
    std::cerr << unknownContacts.toString(model) << std::endl;


    icub_model_estimation->setContacts(input_contact_list);

    icub_model_estimation->kinematicRNEA();
    icub_model_estimation->estimateContactForcesFromSkin();
    ok = icub_model_estimation->dynamicRNEA();

    output_contact_list = icub_model_estimation->getContacts();

    if( output_contact_list.size() != input_contact_list.size() )
    { std::cout << "Error: Estimated a wrong number of contacts."; return EXIT_FAILURE; }

    // Compute the estimated torque with iDynTree
    // Allocate some buffers
    iDynTree::LinkVelArray linkVels(model);
    iDynTree::LinkAccArray linkProperAccs(model);
    iDynTree::estimateExternalWrenchesBuffers extWrenchBufs(subModels);
    iDynTree::LinkInternalWrenches internalWrenches(model);

    // Run the kinematics loop


    iDynTree::dynamicsEstimationForwardVelAccKinematics(model,traversal,
                                              base_classicalProperAcc,base_angularVel,base_angularAcc,
                                              jointPos,jointVel,jointAcc,linkVels,linkProperAccs);

    // Estimate the external forces
    iDynTree::estimateExternalWrenches(model,subModels,sensors,unknownContacts,
                                       jointPos,linkVels,linkProperAccs,
                                       sensMeas,extWrenchBufs,externalWrenches);

    // Convert the external wrenches in a format suitable for RNEA
    externalWrenches.computeNetWrenches(netExternalWrenches);

    // Compute the joint torques
    iDynTree::RNEADynamicPhase(model,traversal,jointPos,linkVels,linkProperAccs,
                                netExternalWrenches,internalWrenches,trqs);

    // Compute the
    dynContactList output_contact_list_with_idyntree;
    iCubSkinHelper.fromiDynTreeToSkinDynLib(model,externalWrenches,output_contact_list_with_idyntree);

    if( output_contact_list_with_idyntree.size() != input_contact_list.size() )
    {
        std::cout << "Error: Estimated a wrong number of contacts in iDynTree.";
        return EXIT_FAILURE;
    }


    // Compare the results
    std::cerr << "Joint torques with KDL/YARP : " << std::endl;
    std::cerr << icub_model_estimation->getTorques().toString() << std::endl;
    std::cerr << "Joint torques with iDynTree: " << std::endl;
    std::cerr << toEigen(trqs.jointTorques()) << std::endl;

    delete icub_model_estimation;

    if( !ok )
    {
        std::cerr << "Error in estimation of joint torques" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
