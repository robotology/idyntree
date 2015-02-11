
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include "test_models.hpp"

#ifndef M_PI_2

#define M_PI_2     1.57079632679489661923132169164    

#endif

namespace KDL {
namespace CoDyCo {

    Tree TestHumanoid(){
        
        Tree humanoid("fake_root_link");
        Chain larm,rarm,torso,head,lleg,rleg,tail;
        
        //Creating random left arm, with embedded ft sensor
        larm.addSegment(Segment("larm_seg0",Joint("larm_jnt0",Joint::RotZ),
                                   Frame::DH(4.0,M_PI_2/2,-3.0,-3.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        larm.addSegment(Segment("larm_seg1",Joint("larm_jnt1",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        larm.addSegment(Segment("larm_seg2",Joint("larm_ft0",Joint::None),
                                   Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                                   RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));
        larm.addSegment(Segment("larm_seg3",Joint("larm_jnt3",Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.4318,0.0),
                                   RigidBodyInertia(0.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
        larm.addSegment(Segment("larm_seg4",Joint("larm_jnt4",Joint::RotZ),
                                   Frame::DH(0.0,-M_PI_2,0.0,0.0),
                                   RigidBodyInertia(0.00,Vector::Zero(),RotationalInertia(0.0,0.0,0.0,0,0,0))));
        larm.addSegment(Segment("larm_seg5",Joint("larm_jnt5",Joint::RotZ),
                                   Frame::DH(0.0,0.0,0.0,0.0),
                                   RigidBodyInertia(0.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
         larm.addSegment(Segment("larm_seg6",Joint("larm_ft1",Joint::None),
                                   Frame::DH(0.0,M_PI_2,0.4318,0.0),
                                   RigidBodyInertia(6.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
        larm.addSegment(Segment("larm_seg7",Joint("larm_jnt7",Joint::RotZ),
                                   Frame::DH(0.0,-M_PI_2,0.0,0.0),
                                   RigidBodyInertia(3.34,Vector::Zero(),RotationalInertia(.3e-3,.4e-3,.3e-3,0,0,0))));
        larm.addSegment(Segment("larm_seg8",Joint("larm_jnt8",Joint::RotZ),
                                   Frame::DH(0.0,0.0,0.0,0.0),
                                   RigidBodyInertia(4.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
        //Creating random right arm
        rarm.addSegment(Segment("rarm_seg0",Joint("rarm_jnt0",Joint::RotZ),
                                   Frame::DH(-4.0,-M_PI_2,4.0,4.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        rarm.addSegment(Segment("rarm_seg1",Joint("rarm_jnt1",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        rarm.addSegment(Segment("rarm_seg2",Joint("rarm_ft0",Joint::None),
                                   Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                                   RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));
        rarm.addSegment(Segment("rarm_seg3",Joint("rarm_jnt3",Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.4318,0.0),
                                   RigidBodyInertia(0.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
        rarm.addSegment(Segment("rarm_seg4",Joint("rarm_jnt4",Joint::RotZ),
                                   Frame::DH(0.0,-M_PI_2,0.0,0.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0.0,0.0,0.0,0,0,0))));
        rarm.addSegment(Segment("rarm_seg5",Joint("rarm_jnt5",Joint::RotZ),
                                   Frame::DH(0.0,0.0,0.0,0.0),
                                   RigidBodyInertia(0.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
        rarm.addSegment(Segment("rarm_seg6",Joint("rarm_ft1",Joint::None),
                                   Frame::DH(0.0,M_PI_2,0.4318,0.0),
                                   RigidBodyInertia(6.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
        rarm.addSegment(Segment("rarm_seg7",Joint("rarm_jnt7",Joint::RotZ),
                                   Frame::DH(0.0,-M_PI_2,0.0,0.0),
                                   RigidBodyInertia(3.34,Vector::Zero(),RotationalInertia(.3e-3,.4e-3,.3e-3,0,0,0))));
        rarm.addSegment(Segment("rarm_seg8",Joint("rarm_jnt8",Joint::RotZ),
                                   Frame::DH(0.0,0.0,0.0,0.0),
                                   RigidBodyInertia(4.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
        
        //Creating random left leg
        lleg.addSegment(Segment("lleg_seg0",Joint("lleg_jnt0",Joint::RotZ),
                                   Frame::DH(-3.0,-M_PI_2/5,3.0,-3.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        lleg.addSegment(Segment("lleg_seg1",Joint("lleg_jnt1",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        lleg.addSegment(Segment("lleg_seg2",Joint("lleg_ft0",Joint::None),
                                   Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                                   RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));
        lleg.addSegment(Segment("lleg_seg3",Joint("lleg_jnt3",Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.4318,0.0),
                                   RigidBodyInertia(0.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
        lleg.addSegment(Segment("lleg_seg4",Joint("lleg_jnt4",Joint::RotZ),
                                   Frame::DH(0.0,-M_PI_2,0.0,0.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0.0,0.0,0.0,0,0,0))));
        lleg.addSegment(Segment("lleg_seg5",Joint("lleg_jnt5",Joint::RotZ),
                                   Frame::DH(0.0,0.0,0.0,0.0),
                                   RigidBodyInertia(0.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
        lleg.addSegment(Segment("lleg_seg6",Joint("lleg_ft1",Joint::None),
                                   Frame::DH(0.0,M_PI_2,0.4318,0.0),
                                   RigidBodyInertia(6.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
        lleg.addSegment(Segment("lleg_seg7",Joint("lleg_jnt7",Joint::RotZ),
                                   Frame::DH(0.0,-M_PI_2,0.0,0.0),
                                   RigidBodyInertia(3.34,Vector::Zero(),RotationalInertia(.3e-3,.4e-3,.3e-3,0,0,0))));
        lleg.addSegment(Segment("lleg_seg8",Joint("lleg_jnt8",Joint::RotZ),
                                   Frame::DH(0.0,0.0,0.0,0.0),
                                   RigidBodyInertia(4.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
        
        //Creating random right leg
        rleg.addSegment(Segment("rleg_seg0",Joint("rleg_jnt0",Joint::RotZ),
                                   Frame::DH(3.0,M_PI_2/3,4.0,4.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        rleg.addSegment(Segment("rleg_seg1",Joint("rleg_jnt1",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        rleg.addSegment(Segment("rleg_seg2",Joint("rleg_ft0",Joint::None),
                                   Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                                   RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));
        rleg.addSegment(Segment("rleg_seg3",Joint("rleg_jnt3",Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.4318,0.0),
                                   RigidBodyInertia(0.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
        rleg.addSegment(Segment("rleg_seg4",Joint("rleg_jnt4",Joint::RotZ),
                                   Frame::DH(0.0,-M_PI_2,0.0,0.0),
                                   RigidBodyInertia(0.00,Vector::Zero(),RotationalInertia(0.00,0.0,0.0,0,0,0))));
        rleg.addSegment(Segment("rleg_seg5",Joint("rleg_jnt5",Joint::RotZ),
                                   Frame::DH(0.0,0.0,0.0,0.0),
                                   RigidBodyInertia(0.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
        rleg.addSegment(Segment("rleg_seg6",Joint("rleg_ft1",Joint::None),
                                   Frame::DH(0.0,M_PI_2,0.4318,0.0),
                                   RigidBodyInertia(6.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
        rleg.addSegment(Segment("rleg_seg7",Joint("rleg_jnt7",Joint::RotZ),
                                   Frame::DH(0.0,-M_PI_2,0.0,0.0),
                                   RigidBodyInertia(3.34,Vector::Zero(),RotationalInertia(.3e-3,.4e-3,.3e-3,0,0,0))));
        rleg.addSegment(Segment("rleg_seg8",Joint("rleg_jnt8",Joint::RotZ),
                                   Frame::DH(0.0,0.0,0.0,0.0),
                                   RigidBodyInertia(4.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
        
        //Creating random torso
        torso.addSegment(Segment("torso_seg0",Joint("torso_jnt0",Joint::None),
                                   Frame::DH(0.0,M_PI_2,0.0,0.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        torso.addSegment(Segment("torso_seg1",Joint("torso_jnt1",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        torso.addSegment(Segment("torso_seg2",Joint("torso_jnt2",Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.0,0.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        torso.addSegment(Segment("torso_seg3",Joint("torso_jnt3",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        
        //Create head
        head.addSegment(Segment("head_seg0",Joint("head_jnt0",Joint::RotZ),
                                   Frame::DH(4.0,M_PI_2,0.5,0.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        head.addSegment(Segment("head_seg1",Joint("head_jnt1",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        head.addSegment(Segment("head_seg2",Joint("head_jnt2",Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.0,0.0),
                                   RigidBodyInertia(4,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        head.addSegment(Segment("head_seg3",Joint("head_ft0",Joint::None),
                                   Frame::DH(3.4,M_PI_2/3,0.0,0.0),
                                   RigidBodyInertia(4,Vector::Zero(),RotationalInertia(2,0.35,5,6,7,0))));
                
        //Aggregate
        humanoid.addChain(torso,"fake_root_link");
        humanoid.addChain(lleg,"torso_seg0");
        humanoid.addChain(rleg,"torso_seg0");
        humanoid.addChain(larm,"torso_seg3");
        humanoid.addChain(rarm,"torso_seg3");
        humanoid.addChain(head,"torso_seg3");

        return humanoid;
    }
    
    
     Tree TestSimpleHumanoid(){
        
        Tree humanoid("fake_root_link");
        Chain larm,rarm,torso,head,lleg,rleg,tail;
        
        //Creating random left arm, with embedded ft sensor
        larm.addSegment(Segment("larm_seg0",Joint("larm_jnt0",Joint::RotZ),
                                   Frame::DH(4.0,M_PI_2/2,-3.0,-3.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        larm.addSegment(Segment("larm_seg1",Joint("larm_jnt1",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        larm.addSegment(Segment("larm_seg2",Joint("larm_ft0",Joint::None),
                                   Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                                   RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));

        //Creating random right arm
        rarm.addSegment(Segment("rarm_seg0",Joint("rarm_jnt0",Joint::RotZ),
                                   Frame::DH(-4.0,-M_PI_2,4.0,4.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        rarm.addSegment(Segment("rarm_seg1",Joint("rarm_jnt1",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
          
        //Creating random left leg
        lleg.addSegment(Segment("lleg_seg0",Joint("lleg_jnt0",Joint::RotZ),
                                   Frame::DH(-3.0,-M_PI_2/5,3.0,-3.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        lleg.addSegment(Segment("lleg_seg1",Joint("lleg_jnt1",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        lleg.addSegment(Segment("lleg_seg2",Joint("lleg_ft0",Joint::None),
                                   Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                                   RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));
        lleg.addSegment(Segment("lleg_seg3",Joint("lleg_jnt3",Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.4318,0.0),
                                   RigidBodyInertia(0.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
              
        //Creating random right leg
        rleg.addSegment(Segment("rleg_seg0",Joint("rleg_jnt0",Joint::RotZ),
                                   Frame::DH(3.0,M_PI_2/3,4.0,4.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        rleg.addSegment(Segment("rleg_seg1",Joint("rleg_jnt1",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
          
        //Creating random torso
        torso.addSegment(Segment("torso_seg0",Joint("torso_jnt0",Joint::None),
                                   Frame::DH(0.0,M_PI_2,0.0,0.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        torso.addSegment(Segment("torso_seg1",Joint("torso_jnt1",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        torso.addSegment(Segment("torso_seg2",Joint("torso_jnt2",Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.0,0.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        torso.addSegment(Segment("torso_seg3",Joint("torso_jnt3",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        
        //Create head
        head.addSegment(Segment("head_seg0",Joint("head_jnt0",Joint::RotZ),
                                   Frame::DH(4.0,M_PI_2,0.5,0.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        head.addSegment(Segment("head_seg1",Joint("head_jnt1",Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
                
        //Aggregate
        humanoid.addChain(torso,"fake_root_link");
        humanoid.addChain(lleg,"torso_seg0");
        humanoid.addChain(rleg,"torso_seg0");
        humanoid.addChain(larm,"torso_seg3");
        humanoid.addChain(rarm,"torso_seg3");
        humanoid.addChain(head,"torso_seg3");

        return humanoid;
    }
}

}