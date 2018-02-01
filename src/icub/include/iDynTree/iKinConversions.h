/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_IKIN_CONVERSIONS_H
#define IDYNTREE_IKIN_CONVERSIONS_H

#include <string>

#include <iCub/iKin/iKinFwd.h>

namespace iDynTree

{

class Model;
class DHChain;

/**
 * \brief Load a iDynTree::DHChain object from a iCub::iKin::iKinChain .
 *
 * @return true if all went ok, false otherwise.
 *
 * \ingroup iDynTreeICUB
 */
bool DHChainFromiKinChain(iCub::iKin::iKinChain & ikinChain,
                          DHChain & out);

/**
 * \brief Load a iDynTree::Model object from a iCub::iKin::iKinChain .
 *
 * @return true if all went ok, false otherwise.
 *
 * \ingroup iDynTreeICUB
 */
bool modelFromiKinChain(iCub::iKin::iKinChain & ikinChain,
                        Model & output);

/**
 * \brief iKinLimb class to extract a iKinLimb from iDynTree structures.
 *
 * \ingroup iDynTreeICUB
 */
class iKinLimbImported : public iCub::iKin::iKinLimb
{
public:
    /**
     * Default constructor.
     */
    iKinLimbImported();

    /**
     * Default destructor.
     */
    virtual ~iKinLimbImported();

    /**
     * Initialize the limb properties from a chain in a iDynTree::Model
     */
    bool fromModel(const Model & model,
                   const std::string& baseFrame,
                   const std::string& distalFrame);

    /**
     * Initialize the limb properties from a iDynTree::DHChain
     */
    bool fromDHChain(const DHChain & dhChain);
};

/**
 * \brief Extract an iCub::iKin::iKinLimb from an iDynTree::Model .
 *
 * @return true if all went ok, false otherwise.
 *
 * \ingroup iDynTreeICUB
 */
bool iKinLimbFromModel(const Model & model,
                        const std::string& baseFrame,
                        const std::string& distalFrame,
                        iCub::iKin::iKinLimb & ikinLimb);

/**
 * \brief Create a iCub::iKin::iKinLimb from an iDynTree::DHChain
 *
 * \ingroup iDynTreeICUB
 */
bool iKinLimbFromDHChain(const DHChain & dhChain,
                         iCub::iKin::iKinLimb& ikinLimb);

}

#endif
