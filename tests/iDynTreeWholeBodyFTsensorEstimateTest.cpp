/** 
* Copyright: 2010-2013 RobotCub Consortium
* Author: Silvio Traversaro, Serena Ivaldi
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
**/ 

//
// This test/example is based on the iDyn tutorial with the same name,
// to show the similarities in the API between iDyn and iDynTree (and 
// for testing
//
// An example on using both iCubTree and iCubWholeBody to estimate the measurements of the FT sensors
// for all the arms and legs, exploiting the modeled dynamic and the inertial sensor 
// measurements.
//
// Author: Silvio Traversaro - <silvio.traversaro@iit.it>
// Author: Serena Ivaldi 

#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/Time.h>
#include <yarp/os/Random.h>

#include <yarp/math/api.h>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <iCub/iDynTree/iCubTree.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iDyn;
using namespace iCub::iDynTree;


void set_random_vector(yarp::sig::Vector & vec, yarp::os::Random & rng, double coeff=1.0)
{
    for( int i=0; i < (int)vec.size(); i++ ) {
        vec[i] =  coeff*M_PI*rng.uniform();
    }
}

int main()
{
    ////////////////////////////////////////////////////////////////////
    //// iDyn
    ////////////////////////////////////////////////////////////////////
    
    
    
    // declare an icub = head + left arm + right arm + torso + left leg + right leg
    // the kinematic and dynamic parameters of each link are automatically set using the 
    // CAD model data.
    // icub default parameters are:
    // mode    = DYNAMIC : the mode for computing wrenches, considering q,dq,d2q,mass,inertia of each link;
    //                     the other main mode is STATIC, which only considers q and mass.
    // verbose = VERBOSE : the verbosity level; I suggest to use VERBOSE during debug, even if you see a lot 
    //                     of messages; then to turn it off with NO_VERBOSE only if everything is really fine;
    //                     if you are using iCubWholeBody you probably don't need it a lot because the inner
    //                     classes have been tested first with iCub, but if you use iDyn in general and use
    //                     it on your own code, it's better to turn verbosity on, it may help with the library
    //                     use. 
    version_tag ver;
    ver.head_version = 1;
    ver.legs_version = 2;
    iCubWholeBody icub(ver);

    // just priting some information to see how the class works
    // iCubWholeBody is basically a container, with Upper and Lower Torso. these two are
    // in fact two public pointers to the iCubUpperTorso and iCubLowerTorso objects, each
    // having all the useful methods for each limb. To access to a specific limb, you must 
    // 'address' it as a string. The string name is quite familiar, it recalls yarp ports..
    cout<<endl
        <<"iCub has many DOF: "<<endl
        <<" - head      : "<<icub.upperTorso->getNLinks("head")<<endl
        <<" - left arm  : "<<icub.upperTorso->getNLinks("left_arm")<<endl
        <<" - right arm : "<<icub.upperTorso->getNLinks("right_arm")<<endl
        <<" - torso     : "<<icub.lowerTorso->getNLinks("torso")<<endl
        <<" - left leg  : "<<icub.lowerTorso->getNLinks("left_leg")<<endl
        <<" - right leg : "<<icub.lowerTorso->getNLinks("right_leg")<<endl<<endl;

    // now we set the joint angles for all the limbs
    // if connected to the real robot, we can take this values from the encoders
    Vector q_head(icub.upperTorso->getNLinks("head"));
    Vector q_larm(icub.upperTorso->getNLinks("left_arm"));
    Vector q_rarm(icub.upperTorso->getNLinks("right_arm"));
    Vector q_torso(icub.lowerTorso->getNLinks("torso"));
    Vector q_lleg(icub.lowerTorso->getNLinks("left_leg"));
    Vector q_rleg(icub.lowerTorso->getNLinks("right_leg"));
    
    //We can initialize the values to random to test that 
    //iDyn and iDynTree return the same results
    yarp::os::Random rng;
    rng.seed((int)yarp::os::Time::now());
    set_random_vector(q_head,rng);
    set_random_vector(q_larm,rng);
    set_random_vector(q_rarm,rng);
    set_random_vector(q_torso,rng);
    set_random_vector(q_lleg,rng);
    set_random_vector(q_rleg,rng);
    
    // here we set the inertial sensor (head) measurements
    Vector w0(3); Vector dw0(3); Vector ddp0(3);
    w0=dw0=ddp0=0.0; ddp0[2]=9.81;

    // here we set the external forces at the end of each limb
    Matrix FM_up(6,3); FM_up.zero();
    Matrix FM_lo(6,2); FM_lo.zero();

    // here we set the joints position, velocity and acceleration 
    // for all the limbs!
    q_head = icub.upperTorso->setAng("head",q_head);
    q_rarm = icub.upperTorso->setAng("right_arm",q_rarm);
    q_larm = icub.upperTorso->setAng("left_arm",q_larm);
    //
    icub.upperTorso->setDAng("head",q_head);
    icub.upperTorso->setDAng("right_arm",q_rarm);
    icub.upperTorso->setDAng("left_arm",q_larm);
    //
    icub.upperTorso->setD2Ang("head",q_head);
    icub.upperTorso->setD2Ang("right_arm",q_rarm);
    icub.upperTorso->setD2Ang("left_arm",q_larm);
    //
    q_torso = icub.lowerTorso->setAng("torso",q_torso);
    q_rleg = icub.lowerTorso->setAng("right_leg",q_rleg);
    q_lleg = icub.lowerTorso->setAng("left_leg",q_lleg);
    //
    icub.lowerTorso->setDAng("torso",q_torso);
    icub.lowerTorso->setDAng("right_leg",q_rleg);
    icub.lowerTorso->setDAng("left_leg",q_lleg);
    //
    icub.lowerTorso->setD2Ang("torso",q_torso);
    icub.lowerTorso->setD2Ang("right_leg",q_rleg);
    icub.lowerTorso->setD2Ang("left_leg",q_lleg);
    
    // initialize the head with the kinematic measures retrieved 
    // by the inertial sensor on the head
    icub.upperTorso->setInertialMeasure(w0,dw0,ddp0);
    
        
    // solve the kinematic of all the limbs attached to the upper torso
    // i.e. head, right arm and left arm
    // then solve the wrench phase, where the external wrenches are set
    // to be acting only on the end-effector of each limb
    // then the limbs with a FT sensor (arms) estimate the sensor wrench
    // note: FM_up = 6x3, because we have 3 limbs with external wrench input
    // note: afterAttach=false is default
    Matrix fm_sens_up = icub.upperTorso->estimateSensorsWrench(FM_up,false);
    
    // connect upper and lower torso: they exchange kinematic and wrench information
    icub.lowerTorso->setInertialMeasure(icub.upperTorso->getTorsoAngVel(),icub.upperTorso->getTorsoAngAcc(),icub.upperTorso->getTorsoLinAcc());

 
    
    // now solve lower torso the same way of the upper
    // note: FM_lo = 6x2, because we have 2 limbs with external wrench input to be set
    // since the torso receives the external wrench output from the upperTorso node
    // during the attachTorso()
    // note: afterAttach=false is set to true, because we specify that the torso
    // already received the wrench for initialization by the upperTorso during
    // the attachTorso()
    Matrix fm_sens_lo = icub.lowerTorso->estimateSensorsWrench(FM_lo);

    cout<<endl
        <<"Estimate FT sensor measurements on upper body: "<<endl
        <<" left  : "<<fm_sens_up.getCol(1).toString()<<endl
        <<" right : "<<fm_sens_up.getCol(0).toString()<<endl
        <<endl
        <<"Estimate FT sensor measurements on lower body: "<<endl
        <<" left  : "<<fm_sens_lo.getCol(1).toString()<<endl
        <<" right : "<<fm_sens_lo.getCol(0).toString()<<endl
        <<endl
        <<"Torques in upper body" << endl
        <<" head : " << icub.upperTorso->getTorques("head").toString() << endl
        <<" left arm: " << icub.upperTorso->getTorques("left_arm").toString() << endl
        <<" right arm: " << icub.upperTorso->getTorques("right_arm").toString() << endl
        <<"Torques in lower body" << endl
        <<" torso : " << icub.lowerTorso->getTorques("torso").toString() << endl
        <<" left leg: " << icub.lowerTorso->getTorques("left_leg").toString() << endl
        <<" right leg: " << icub.lowerTorso->getTorques("right_leg").toString() << endl;



    ////////////////////////////////////////////////////////////////////
    //// iDyn
    ////////////////////////////////////////////////////////////////////
    
    
    
    // declare an iCubTree
    iCubTree_version_tag version;
    version.head_version = ver.head_version;
    version.legs_version = ver.legs_version;
    
    //Use the same serialization as used in IDYN_SERIALIZATION
    iCubTree icub_tree(version,IDYN_SERIALIZATION);

    // just priting some information to see how the class works
    cout<<endl
        <<"iCub has many DOF: "<<endl
        <<" - head      : "<<icub_tree.getNrOfDOFs("head")<<endl
        <<" - left arm  : "<<icub_tree.getNrOfDOFs("left_arm")<<endl
        <<" - right arm : "<<icub_tree.getNrOfDOFs("right_arm")<<endl
        <<" - torso     : "<<icub_tree.getNrOfDOFs("torso")<<endl
        <<" - left leg  : "<<icub_tree.getNrOfDOFs("left_leg")<<endl
        <<" - right leg : "<<icub_tree.getNrOfDOFs("right_leg")<<endl<<endl;


    // here we set the joints position, velocity and acceleration 
    // for all the limbs!
    icub_tree.setAng(q_head,"head");
    icub_tree.setAng(q_rarm,"right_arm");
    icub_tree.setAng(q_larm,"left_arm");
    //
    icub_tree.setDAng(q_head,"head");
    icub_tree.setDAng(q_rarm,"right_arm");
    icub_tree.setDAng(q_larm,"left_arm");
    //
    icub_tree.setD2Ang(q_head,"head");
    icub_tree.setD2Ang(q_rarm,"right_arm");
    icub_tree.setD2Ang(q_larm,"left_arm");
    //
    icub_tree.setAng(q_torso,"torso");
    icub_tree.setAng(q_rleg,"right_leg");
    icub_tree.setAng(q_lleg,"left_leg");
    //
    icub_tree.setDAng(q_torso,"torso");
    icub_tree.setDAng(q_rleg,"right_leg");
    icub_tree.setDAng(q_lleg,"left_leg");
    //
    icub_tree.setD2Ang(q_torso,"torso");
    icub_tree.setD2Ang(q_rleg,"right_leg");
    icub_tree.setD2Ang(q_lleg,"left_leg");
    
    // initialize the head with the kinematic measures retrieved 
    // by the inertial sensor on the head
    icub_tree.setInertialMeasure(w0,dw0,ddp0);
    
    icub_tree.kinematicRNEA();
    icub_tree.dynamicRNEA();
    
    Vector left_arm_ft(6), right_arm_ft(6), left_leg_ft(6), right_leg_ft(6);
    icub_tree.getSensorMeasurement(0,left_arm_ft);
    icub_tree.getSensorMeasurement(1,right_arm_ft);
    icub_tree.getSensorMeasurement(2,left_leg_ft);
    icub_tree.getSensorMeasurement(3,right_leg_ft);
    
    Vector com(3);
    double mass;
    icub.computeCOM();
    icub.getCOM(iCub::skinDynLib::BODY_PART_ALL,com,mass);

    cout<<endl
    <<"Estimate FT sensor measurements on upper body: "<<endl
    <<" left  : "<<left_arm_ft.toString()<<endl
    <<" right : "<<right_arm_ft.toString()<<endl
    <<endl
    <<"Estimate FT sensor measurements on lower body: "<<endl
    <<" left  : "<<left_leg_ft.toString()<<endl
    <<" right : "<<right_leg_ft.toString()<<endl
    <<endl
    
    <<"Torques in upper body" << endl
    <<" head : " << icub_tree.getTorques("head").toString() << endl
    <<" left arm: " << icub_tree.getTorques("left_arm").toString() << endl
    <<" right arm: " << icub_tree.getTorques("right_arm").toString() << endl
    <<"Torques in lower body" << endl
    <<" torso : " << icub_tree.getTorques("torso").toString() << endl
    <<" left leg: " << icub_tree.getTorques("left_leg").toString() << endl
    <<" right leg: " << icub_tree.getTorques("right_leg").toString() << endl;

    
    
    cout <<"Difference in estimate FT sensor measurements on upper body: "<<endl
    <<" left  : "<<(fm_sens_up.getCol(1) -left_arm_ft).toString()<<endl
    <<" right : "<<(fm_sens_up.getCol(0) -right_arm_ft).toString()<<endl
    <<endl
    <<"Difference in estimate FT sensor measurements on lower body: "<<endl
    <<" left  : "<<(fm_sens_lo.getCol(1)-left_leg_ft).toString()<<endl
    <<" right : "<<(fm_sens_lo.getCol(0)-right_leg_ft).toString()<<endl
    <<endl
    

    <<"Difference in torques in upper body" << endl
    <<" head : " << (icub.upperTorso->getTorques("head").subVector(0,2)-icub_tree.getTorques("head")).toString() << endl
    <<" left arm: " << (icub.upperTorso->getTorques("left_arm")-icub_tree.getTorques("left_arm")).toString() << endl
    <<" right arm: " << (icub.upperTorso->getTorques("right_arm")-icub_tree.getTorques("right_arm")).toString() << endl
    <<"Difference in torques in lower body" << endl
    <<" torso : " << (icub.lowerTorso->getTorques("torso")-icub_tree.getTorques("torso")).toString() << endl
    <<" left leg: " <<  (icub.lowerTorso->getTorques("left_leg")-icub_tree.getTorques("left_leg")).toString() << endl
    <<" right leg: " <<  (icub.lowerTorso->getTorques("right_leg")-icub_tree.getTorques("right_leg")).toString() << endl

    <<"Difference in positions in upper body" << endl
    <<" head : " << (icub.upperTorso->getAng("head").subVector(0,2)-icub_tree.getAng("head")).toString() << endl
    <<" left arm: " << (icub.upperTorso->getAng("left_arm")-icub_tree.getAng("left_arm")).toString() << endl
    <<" right arm: " << (icub.upperTorso->getAng("right_arm")-icub_tree.getAng("right_arm")).toString() << endl
    <<"Difference in positioon in lower body" << endl
    <<" torso : " << (icub.lowerTorso->getAng("torso")-icub_tree.getAng("torso")).toString() << endl
    <<" left leg: " <<  (icub.lowerTorso->getAng("left_leg")-icub_tree.getAng("left_leg")).toString() << endl
    <<" right leg: " <<  (icub.lowerTorso->getAng("right_leg")-icub_tree.getAng("right_leg")).toString() << endl

    << "Difference in COM " << (com-icub_tree.getCOM()).toString() << std::endl
    << "Original COM " << com.toString() << std::endl
    << "iDynTree COM " << ((icub_tree.getCOM())).toString() << std::endl;
    
    return 0;
}
      
      

