/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#include <iCub/iDynTree/iDyn2KDL.h>

bool idynChain2kdlChain(iCub::iDyn::iDynChain & idynChain,KDL::Chain & kdlChain,std::vector<std::string> link_names,std::vector<std::string> joint_names, std::string final_frame_name, std::string initial_frame_name, int max_links)
{
    int n_links, i;
    bool use_names;
    n_links = idynChain.getN();
    //In iDyn, the joints are only 1 DOF (not 0, not more)
    int n_joints = idynChain.getN();
    
    if(n_links <= 0 ) return false;
    
    if( (int)link_names.size() < n_links || (int)joint_names.size() < n_joints ) {
        use_names = false;
    } else {
        use_names = true;
    }
    
    
    KDL::Frame kdlFrame = KDL::Frame();
    KDL::Frame kdl_H = KDL::Frame();
    KDL::Joint kdlJoint = KDL::Joint();
    kdlChain = KDL::Chain();
    KDL::Segment kdlSegment = KDL::Segment();
    KDL::RigidBodyInertia kdlRigidBodyInertia = KDL::RigidBodyInertia();
    
        
    if( initial_frame_name.length() != 0 ) {
            idynMatrix2kdlFrame(idynChain.getH0(),kdlFrame);
            kdlSegment = KDL::Segment(initial_frame_name,KDL::Joint(initial_frame_name+"_joint",KDL::Joint::None),kdlFrame);
            kdlChain.addSegment(kdlSegment);
    }
    
    
    for(i=0;i<n_links && i < max_links;i++) 
    {
        //forgive him, as he does not know what is doing
        iCub::iKin::iKinLink & link_current = idynChain[i];
        //For the first link and the last link, take in account also H0 and HN
        if ( i == n_links - 1) {
            if( final_frame_name.length() == 0 ) {
                idynMatrix2kdlFrame(idynChain.getHN(),kdl_H);
                kdlFrame = kdlFrame.DH(link_current.getA(),link_current.getAlpha(),link_current.getD(),link_current.getOffset())*kdl_H;
            } else {
                kdlFrame = kdlFrame.DH(link_current.getA(),link_current.getAlpha(),link_current.getD(),link_current.getOffset());
            }
        } else {
            kdlFrame = kdlFrame.DH(link_current.getA(),link_current.getAlpha(),link_current.getD(),link_current.getOffset());
        }

        
        bool ret = idynDynamicalParameters2kdlRigidBodyInertia(idynChain.getMass(i),idynChain.getCOM(i).subcol(0,3,3),idynChain.getInertia(i),kdlRigidBodyInertia);        
        if( !ret ) return false;
        
        KDL::Joint jnt_idyn;
        
        //\todo: joint can also be blocked at values different from 0.0
        if( idynChain.isLinkBlocked(i) ) {
            if( use_names ) {
                kdlSegment = KDL::Segment(link_names[i],KDL::Joint(joint_names[i],KDL::Joint::None),kdlFrame,kdlRigidBodyInertia);
            } else {   
                kdlSegment = KDL::Segment(KDL::Joint(KDL::Joint::None),kdlFrame,kdlRigidBodyInertia);
            }
        } else {
            if( use_names ) {
                kdlSegment = KDL::Segment(link_names[i],KDL::Joint(joint_names[i],KDL::Joint::RotZ),kdlFrame,kdlRigidBodyInertia);
            } else {   
                kdlSegment = KDL::Segment(KDL::Joint(KDL::Joint::RotZ),kdlFrame,kdlRigidBodyInertia);
            }
        }
        kdlChain.addSegment(kdlSegment);
        

    }
    
    //if specified, add a final fake link 
    if( final_frame_name.length() != 0 ) {
        idynMatrix2kdlFrame(idynChain.getHN(),kdlFrame);
        kdlSegment = KDL::Segment(final_frame_name,KDL::Joint(final_frame_name+"_joint",KDL::Joint::None),kdlFrame);
        kdlChain.addSegment(kdlSegment);
    }
    
    //Considering the H0 transformation
    if( initial_frame_name.length() == 0 ) {
        KDL::Chain new_chain;
        KDL::Frame kdl_H0;
        idynMatrix2kdlFrame(idynChain.getH0(),kdl_H0);
        //std::cout << "KDL_h0 " <<  kdl_H0 << std::endl;
    
        addBaseTransformation(kdlChain,new_chain,kdl_H0);
        kdlChain = new_chain;
    }
        
    return true;
}

bool idynSensorChain2kdlChain(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor ,KDL::Chain & kdlChain, std::vector<std::string> link_names,std::vector<std::string> joint_names, std::string final_frame_name, std::string initial_frame_name, int max_links)
{
    bool use_names;
    int n_links, i, sensor_link;
    n_links = idynChain.getN();
    sensor_link = idynSensor.getSensorLink();
    
    int kdl_links = n_links + 1; //The sensor links transform a link in two different links
    int kdl_joints = kdl_links;

    if(n_links <= 0 ) return false;
    
    if( (int)link_names.size() < kdl_links || (int)joint_names.size() < kdl_joints ) {
        use_names = false;
    } else {
        use_names = true;
    }
    
    
    KDL::Frame kdlFrame = KDL::Frame();
    KDL::Frame kdl_H;
    kdlChain = KDL::Chain();
    KDL::Segment kdlSegment = KDL::Segment();
    KDL::RigidBodyInertia kdlRigidBodyInertia = KDL::RigidBodyInertia();
    
    int kdl_i = 0;
    
    if( initial_frame_name.length() != 0 ) {
            idynMatrix2kdlFrame(idynChain.getH0(),kdlFrame);
            kdlSegment = KDL::Segment(initial_frame_name,KDL::Joint(initial_frame_name+"_joint",KDL::Joint::None),kdlFrame);
            kdlChain.addSegment(kdlSegment);
    }
    
    for(i=0;i<n_links;i++) 
    {
        if( i != sensor_link ) {
            //forgive him, as he does not know what is doing
            iCub::iKin::iKinLink & link_current = idynChain[i];
            //For the first link and the last link, take in account also H0 and HN
            if ( i == n_links - 1) {
                idynMatrix2kdlFrame(idynChain.getHN(),kdl_H);
                kdlFrame = kdlFrame.DH(link_current.getA(),link_current.getAlpha(),link_current.getD(),link_current.getOffset())*kdl_H;
            } else {
                kdlFrame = kdlFrame.DH(link_current.getA(),link_current.getAlpha(),link_current.getD(),link_current.getOffset());
            }

            

            bool ret = idynDynamicalParameters2kdlRigidBodyInertia(idynChain.getMass(i),idynChain.getCOM(i).subcol(0,3,3),idynChain.getInertia(i),kdlRigidBodyInertia);
            assert(ret);
            if(!ret) return false;
            
            if( idynChain.isLinkBlocked(i) ) {
                if( use_names ) {
                    kdlSegment = KDL::Segment(link_names[kdl_i],KDL::Joint(joint_names[kdl_i],KDL::Joint::None),kdlFrame,kdlRigidBodyInertia);
                    kdl_i++;
                } else {
                    kdlSegment = KDL::Segment(KDL::Joint(KDL::Joint::None),kdlFrame,kdlRigidBodyInertia);
                }
            } else {
                if( use_names ) {
                    kdlSegment = KDL::Segment(link_names[kdl_i],KDL::Joint(joint_names[kdl_i],KDL::Joint::RotZ),kdlFrame,kdlRigidBodyInertia);
                    kdl_i++;
                } else {
                    kdlSegment = KDL::Segment(KDL::Joint(KDL::Joint::RotZ),kdlFrame,kdlRigidBodyInertia);
                }
            }
            kdlChain.addSegment(kdlSegment);
        } else {
        //( i == segment_link )
            double m,m0,m1;
            yarp::sig::Vector r0(3),r1(3),r(3),rgg0(3),rgg1(3),r_i_s_wrt_i(3),r_i_C0_wrt_i;
            yarp::sig::Matrix I,I0,I1;
            yarp::sig::Matrix R_s_wrt_i;
            
            iCub::iKin::iKinLink & link_current = idynChain[sensor_link];
            KDL::Frame kdlFrame_0 = KDL::Frame();
            KDL::Frame kdlFrame_1 = KDL::Frame();
            
            //Imagine that we have i, s , i+1
            
            //yarp::sig::Matrix H_0;
            //The angle of the sensor link joint is put to 0 and then restored
            double angSensorLink = link_current.getAng();
            yarp::sig::Matrix  H_sensor_link = (link_current.getH(0.0)); //H_{i-1}_i
            link_current.setAng(angSensorLink);
            //idynSensor.getH() <--> H_i_s
            yarp::sig::Matrix H_0 = H_sensor_link  * (idynSensor.getH()); // H_{i-1}_s = H_{i-1}_i*H_i_s ?  
            yarp::sig::Matrix H_1 = localSE3inv(idynSensor.getH()); //H_s_{i} 
            //std::cout << "H_0" << std::endl << H_0.toString() << std::endl;
            //std::cout << "H_1" << std::endl << H_1.toString() << std::endl;
            idynMatrix2kdlFrame(H_0,kdlFrame_0);
            idynMatrix2kdlFrame(H_1,kdlFrame_1);
            //cout << "kdlFrame_0: " << endl;
            //cout << kdlFrame_0;
            //cout << "kdlFrame_1: " << endl;
            //cout << kdlFrame_1; 

            
            KDL::RigidBodyInertia kdlRigidBodyInertia_0 = KDL::RigidBodyInertia();
            KDL::RigidBodyInertia kdlRigidBodyInertia_1 = KDL::RigidBodyInertia();
            
            m = idynChain.getMass(sensor_link);
            m1 = idynSensor.getMass();
            m0 = m-m1;
            //It is not possible that the semilink is more heavy then the link!!!
            assert(m0 > 0);
            
            //r_{i,C_i}^i
            r = idynChain.getCOM(i).subcol(0,3,3);
            
            //R_s^i
            R_s_wrt_i = idynSensor.getH().submatrix(0,2,0,2);
            
            r_i_s_wrt_i = idynSensor.getH().subcol(0,3,3);

            //r0 := r_{s,C_{{vl}_0}}^s
            //r1 := r_{i,C_{{vl}_1}}^i
            
            
            
            //
            r1 = r_i_s_wrt_i +  R_s_wrt_i*(idynSensor.getCOM().subcol(0,3,3));
            
            //cout << "m0: " << m0 << endl;
            r_i_C0_wrt_i = (1/m0)*(m*r-m1*r1);
            
            r0 = R_s_wrt_i.transposed()*(r_i_C0_wrt_i-r_i_s_wrt_i);

            I = idynChain.getInertia(i);
            
            I1 = R_s_wrt_i*idynSensor.getInertia()*R_s_wrt_i.transposed();
            rgg0 = -1*r+r_i_C0_wrt_i;
            rgg1 = -1*r+r1;

            I0 = R_s_wrt_i.transposed()*(I - I1 + m1*crossProductMatrix(rgg1)*crossProductMatrix(rgg1) +  m0*crossProductMatrix(rgg0)*crossProductMatrix(rgg0))*R_s_wrt_i;

            //DEBUG
            //printMatrix("I",I);
            //printMatrix("I1",I1);
            //printMatrix("I0",I0);
            //printMatrix("cross",crossProductMatrix(rgg1));
            
            //cout << "m0: " << m0 << endl;
            //printVector("r0",r0);
            //printMatrix("I0",I0);
            bool ret;
            ret = idynDynamicalParameters2kdlRigidBodyInertia(m0,r0,I0,kdlRigidBodyInertia_0);
            assert(ret);
            if(!ret) return false;
            ret = idynDynamicalParameters2kdlRigidBodyInertia(m1,r1,I1,kdlRigidBodyInertia_1);
            assert(ret);
            if(!ret) return false;

            if( use_names ) {
                kdlSegment = KDL::Segment(link_names[kdl_i],KDL::Joint(joint_names[kdl_i],KDL::Joint::RotZ),kdlFrame_0,kdlRigidBodyInertia_0);
                kdl_i++;
            } else {
                kdlSegment = KDL::Segment(KDL::Joint(KDL::Joint::RotZ),kdlFrame_0,kdlRigidBodyInertia_0);
            }
            kdlChain.addSegment(kdlSegment);

            if( use_names ) {
                kdlSegment = KDL::Segment(link_names[kdl_i],KDL::Joint(joint_names[kdl_i],KDL::Joint::None),kdlFrame_1,kdlRigidBodyInertia_1);
                kdl_i++;
            } else {
                kdlSegment = KDL::Segment(KDL::Joint(KDL::Joint::None),kdlFrame_1,kdlRigidBodyInertia_1);
            }
            kdlChain.addSegment(kdlSegment);
        }
        
    }
    //Final link can be segment
    if( final_frame_name.length() != 0 ) {
        idynMatrix2kdlFrame(idynChain.getHN(),kdlFrame);
        kdlSegment = KDL::Segment(final_frame_name,KDL::Joint(final_frame_name+"_joint",KDL::Joint::None),kdlFrame);
        kdlChain.addSegment(kdlSegment);
    }
    
    if( max_links < (int)kdlChain.getNrOfSegments() ) 
    {
        KDL::Chain new_kdlChain;
        for(int p=0;p < max_links;p++) 
        {
            new_kdlChain.addSegment(kdlChain.getSegment(p));
        }
        kdlChain = new_kdlChain;
    }

    //Considering the H0 transformation
    if( initial_frame_name.length() == 0 ) {
        KDL::Chain new_chain;
        KDL::Frame kdl_H0;
        idynMatrix2kdlFrame(idynChain.getH0(),kdl_H0);
        addBaseTransformation(kdlChain,new_chain,kdl_H0);
        kdlChain = new_chain;
    }
    
    return true;
}

bool idynDynamicalParameters2kdlRigidBodyInertia(const double idynmass,const yarp::sig::Vector & idynrC,const yarp::sig::Matrix & idynI,KDL::RigidBodyInertia & kdlRigidBodyInertia)
{
    

    if(idynrC.size() != 3 || idynI.cols() != 3 || idynI.rows() != 3 ) return false;
    KDL::Vector kdlrC;
    KDL::RotationalInertia kdlRotationalInertia;
    
    int ret = idynVector2kdlVector(idynrC,kdlrC);
    assert(ret);
    //printMatrix("idynI",idynI);
    if( !idynInertia2kdlRotationalInertia(idynI,kdlRotationalInertia) ) return false;

    kdlRigidBodyInertia = KDL::RigidBodyInertia(idynmass,kdlrC,kdlRotationalInertia);
    return true;
    
}

bool idynInertia2kdlRotationalInertia(const yarp::sig::Matrix & idynInertia,KDL::RotationalInertia & kdlRotationalInertia)
{
     if(idynInertia.cols() != 3 || idynInertia.rows() != 3 ) return false;
     //if(idynInertia(0,1) != idynInertia(1,0) || idynInertia(0,2) != idynInertia(2,0) || idynInertia(1,2) != idynInertia(2,1)) return false;
     kdlRotationalInertia = KDL::RotationalInertia(idynInertia(0,0),idynInertia(1,1),idynInertia(2,2),idynInertia(0,1),idynInertia(0,2),idynInertia(1,2));
     return true;
}

bool idynMatrix2kdlFrame(const yarp::sig::Matrix & idynMatrix, KDL::Frame & kdlFrame)
{
    if( idynMatrix.cols() != 4 || idynMatrix.rows() != 4 ) return false;
    KDL::Rotation kdlRotation;
    KDL::Vector kdlVector;
    idynMatrix2kdlRotation(idynMatrix.submatrix(0,2,0,2),kdlRotation);
    idynVector2kdlVector(idynMatrix.subcol(0,3,3),kdlVector);
    kdlFrame = KDL::Frame(kdlRotation,kdlVector);
    return true;
}

bool idynMatrix2kdlRotation(const yarp::sig::Matrix & idynMatrix, KDL::Rotation & kdlRotation)
{
    if(idynMatrix.cols() != 3 || idynMatrix.rows() != 3) return false;
    kdlRotation = KDL::Rotation(idynMatrix(0,0),idynMatrix(0,1),idynMatrix(0,2),
                                idynMatrix(1,0),idynMatrix(1,1),idynMatrix(1,2),
                                idynMatrix(2,0),idynMatrix(2,1),idynMatrix(2,2));
    return true;
}

bool idynVector2kdlVector(const yarp::sig::Vector & idynVector, KDL::Vector & kdlVector)
{
    if( idynVector.size() != 3 ) return false;
    kdlVector = KDL::Vector(idynVector[0],idynVector[1],idynVector[2]);
    return true;
}


void printKDLchain(std::string s,const KDL::Chain & kdlChain)
{
    std::cout << s << std::endl;
    for(int p=0;p < (int)kdlChain.getNrOfSegments();p++)
    {
        std::cout << "Segment " << p << ":" << std::endl;
        KDL::Segment seg = kdlChain.getSegment(p);
        KDL::Frame fra = seg.getFrameToTip();
        std::cout << seg << std::endl;
        std::cout << fra << std::endl;
    }
}


/************************************************************************/
yarp::sig::Matrix localSE3inv(const yarp::sig::Matrix &H, unsigned int verbose)
{    
    if ((H.rows()<4) || (H.cols()<4))
    {
        if (verbose)
            fprintf(stderr,"localSE3inv() failed\n");

        return yarp::sig::Matrix(0,0);
    }

    yarp::sig::Matrix invH(4,4);
    yarp::sig::Vector p(3);

    yarp::sig::Matrix Rt=H.submatrix(0,2,0,2).transposed();
    p[0]=H(0,3);
    p[1]=H(1,3);
    p[2]=H(2,3);

    p=Rt*p;

    for (unsigned int i=0; i<3; i++)
        for (unsigned int j=0; j<3; j++)
            invH(i,j)=Rt(i,j);

    invH(0,3)=-p[0];
    invH(1,3)=-p[1];
    invH(2,3)=-p[2];
    invH(3,3)=1.0;

    return invH;
}


bool addBaseTransformation(const KDL::Chain & old_chain, KDL::Chain & new_chain, KDL::Frame H_new_old)
{
    new_chain = KDL::Chain();
    for(unsigned int i=0;i<old_chain.getNrOfSegments();i++){
        KDL::Segment segm;
        segm = old_chain.getSegment(i);
        //if is not the first segment add normally the segment
        if( i != 0 ) { 
            new_chain.addSegment(segm);
        } else {
            //otherwise modify the segment before adding it
            KDL::Segment new_segm;
            KDL::Joint new_joint, old_joint;
            old_joint = segm.getJoint();
            KDL::Joint::JointType new_type;
            switch(old_joint.getType()) {
                case KDL::Joint::RotAxis:
                case KDL::Joint::RotX:
                case KDL::Joint::RotY:
                case KDL::Joint::RotZ:
                    new_type = KDL::Joint::RotAxis;
                break;
                case KDL::Joint::TransAxis:
                case KDL::Joint::TransX:
                case KDL::Joint::TransY:
                case KDL::Joint::TransZ:
                    new_type = KDL::Joint::TransAxis;
                break; 
                case KDL::Joint::None:
                default:
                    new_type = KDL::Joint::None;
            }
            
            //check !
        
            new_joint = KDL::Joint(old_joint.getName(),H_new_old*old_joint.JointOrigin(),H_new_old.M*old_joint.JointAxis(),new_type);
            new_segm = KDL::Segment(segm.getName(),new_joint,H_new_old*segm.getFrameToTip(),segm.getInertia());
            new_chain.addSegment(new_segm);
        }
    }
    return true;
}
