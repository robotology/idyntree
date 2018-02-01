/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
