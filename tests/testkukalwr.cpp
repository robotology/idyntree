
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include "test_models.hpp"

#ifndef M_PI_2

#define M_PI_2     1.57079632679489661923132169164    

#endif

namespace KDL {
namespace CoDyCo {

    Tree TestKukaLWR(){
        
        Tree single_link("fake_root_link");
        
        Chain kukaLWR_DHnew;

        
        kukaLWR_DHnew.addSegment(Segment("seg0",Joint("jnt0",Joint::None),
                  Frame::DH_Craig1989(4.0, 3.0, 0.31, 2.0), RigidBodyInertia(3)
                  ));
        //joint 1
        kukaLWR_DHnew.addSegment(Segment("seg1",Joint("jnt1",Joint::RotZ),
                  Frame::DH_Craig1989(4.0, 1.5707963, 2.0, -3.0),
                  Frame::DH_Craig1989(-5.0, 1.5707963, 2.0, -3.0).Inverse()*RigidBodyInertia(1,
                                                 Vector::Zero(),
                                                 RotationalInertia(0.0,0.0,0.0115343,0.0,0.0,0.0))));
                   
    //joint 2 
    kukaLWR_DHnew.addSegment(Segment("seg2",Joint("jnt2",Joint::RotZ),
                  Frame::DH_Craig1989(-4.0, -1.5707963, 0.4, 3.0),
                  Frame::DH_Craig1989(-5.0, -1.5707963, 0.4, -5.0).Inverse()*RigidBodyInertia(4,
                                                   Vector(0.0,-0.3120511,-0.0038871),
                                                   RotationalInertia(-0.5471572,-0.0000302,-0.5423253,0.0,0.0,0.0018828))));
                  
    //joint 3
    kukaLWR_DHnew.addSegment(Segment("seg3",Joint("jnt3",Joint::RotZ),
                  Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                  Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                   Vector(0.0,-0.0015515,0.0),
                                                   RotationalInertia(0.0063507,0.0,0.0107804,0.0,0.0,-0.0005147))));
                  
    //joint 4
    kukaLWR_DHnew.addSegment(Segment("seg4",Joint("jtn4",Joint::RotZ),
                  Frame::DH_Craig1989(4.0, 1.5707963, 0.39, 0.0),
                  Frame::DH_Craig1989(4.0, 1.5707963, 0.39, 0.0).Inverse()*RigidBodyInertia(2,
                                                   Vector(0.0,0.5216809,0.0),
                                                   RotationalInertia(-1.0436952,0.0,-1.0392780,0.0,0.0,0.0005324))));
                  
    //joint 5
    kukaLWR_DHnew.addSegment(Segment("seg5",Joint("jnt5",Joint::RotZ),
                  Frame::DH_Craig1989(0.0, 1.5707963, 4.0, 0.0),
                  Frame::DH_Craig1989(0.0, 1.5707963, 4.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                   Vector(0.0,0.0119891,0.0),
                                                   RotationalInertia(0.0036654,0.0,0.0060429,0.0,0.0,0.0004226))));
                  
    //joint 6
    kukaLWR_DHnew.addSegment(Segment("seg6",Joint("jnt6",Joint::RotZ),
                  Frame::DH_Craig1989(0.0, -1.5707963, -4.0, 0.0),
                  Frame::DH_Craig1989(0.0, -1.5707963, -4.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                   Vector(0.0,0.0080787,0.0),
                                                   RotationalInertia(0.0010431,0.0,0.0036376,0.0,0.0,0.0000101))));
    //joint 7
    kukaLWR_DHnew.addSegment(Segment("seg7",Joint("seg7",Joint::RotZ),
                   Frame::Identity(),
                   RigidBodyInertia(10,
                                                   Vector::Zero(),
                                                   RotationalInertia(0.000001,0.0,0.0001203,0.0,0.0,0.0))));

        single_link.addChain(kukaLWR_DHnew,"fake_root_link");
        return single_link;
    }
    
}

}