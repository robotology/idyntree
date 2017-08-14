/*
 * Copyright (C) 2017 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#include <iDynTree/iKinConversions.h>
#include <iDynTree/Model/DenavitHartenberg.h>

#include <iDynTree/yarp/YARPConversions.h>

#include <iCub/iKin/iKinFwd.h>

#include <cassert>

namespace iDynTree
{

DHLink iKinLink2DHLink(const iCub::iKin::iKinLink & ikinlink)
{
    DHLink ret;

    ret.A = ikinlink.getA();
    ret.D = ikinlink.getD();
    ret.Alpha = ikinlink.getAlpha();
    ret.Offset = ikinlink.getOffset();
    ret.Min = ikinlink.getMin();
    ret.Max = ikinlink.getMax();

    return ret;
}

iCub::iKin::iKinLink DHLink2iKinLink(const DHLink & dhLink)
{
    return iCub::iKin::iKinLink(dhLink.A,dhLink.D,
                                dhLink.Alpha,dhLink.Offset,
                                dhLink.Min,dhLink.Max);
}

bool DHChainFromiKinChain(iCub::iKin::iKinChain& ikinChain,
                                DHChain& dhChain)
{
    assert(ikinChain.getN() == ikinChain.getDOF());

    iDynTree::Transform H0, HN;
    dhChain.setNrOfDOFs(ikinChain.getN());

    toiDynTree(ikinChain.getH0(),H0);
    dhChain.setH0(H0);

    for(size_t i=0; i < dhChain.getNrOfDOFs(); i++)
    {
        dhChain(i) = iKinLink2DHLink(ikinChain(i));
        std::stringstream ss;
        ss << i;
        dhChain.setDOFName(i, "iKinDOF" + ss.str());
    }

    toiDynTree(ikinChain.getHN(),HN);
    dhChain.setHN(HN);

    return true;
}

bool modelFromiKinChain(iCub::iKin::iKinChain& ikinChain, Model& output)
{
    DHChain chain;
    bool ok = DHChainFromiKinChain(ikinChain, chain);

    if( !ok )
    {
        return false;
    }

    ok = CreateModelFromDHChain(chain,output);

    return ok;
}

iKinLimbImported::iKinLimbImported(): iCub::iKin::iKinLimb()
{
}

iKinLimbImported::~iKinLimbImported()
{}

bool iKinLimbImported::fromDHChain(const DHChain &dhChain)
{
    // Cleanup existing data
    dispose();

    yarp::sig::Matrix yarpMatBuf;
    toYarp(dhChain.getH0().asHomogeneousTransform(), yarpMatBuf);
    this->setH0(yarpMatBuf);

    for(size_t i=0; i < dhChain.getNrOfDOFs(); i++)
    {
        // The ownership of the pointer is transferred to the iKinLimb object,
        // and it will be freed by the denstructor (or a call to dispose)
        this->pushLink(new iCub::iKin::iKinLink(DHLink2iKinLink(dhChain(i))));
    }

    toYarp(dhChain.getHN().asHomogeneousTransform(), yarpMatBuf);
    this->setHN(yarpMatBuf);

    return true;
}

bool iKinLimbImported::fromModel(const Model &model,
                                 const std::string &baseFrame,
                                 const std::string &distalFrame)
{
    DHChain chain;
    bool conversionSuccessful = ExtractDHChainFromModel(model, baseFrame, distalFrame, chain);
    conversionSuccessful = conversionSuccessful && this->fromDHChain(chain);
    return conversionSuccessful;
}

bool iKinLimbFromDHChain(const DHChain & dhChain,
                         iCub::iKin::iKinLimb& ikinLimb)
{
    iKinLimbImported ikinLimbImported;

    bool ok = ikinLimbImported.fromDHChain(dhChain);

    ikinLimb = ikinLimbImported;

    return ok;
}

bool iKinLimbFromModel(const Model & model,
                       const std::string& baseFrame,
                       const std::string& distalFrame,
                       iCub::iKin::iKinLimb & ikinLimb)
{
    DHChain chain;
    bool conversionSuccessful = ExtractDHChainFromModel(model, baseFrame, distalFrame, chain);
    conversionSuccessful = conversionSuccessful && iKinLimbFromDHChain(chain, ikinLimb);
    return conversionSuccessful;
}

}
