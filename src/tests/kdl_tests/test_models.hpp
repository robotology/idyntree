/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#ifndef KDL_CODYCO_TEST_MODELS_HPP
#define KDL_CODYCO_TEST_MODELS_HPP

namespace KDL{

class Tree;
    
namespace CoDyCo {


Tree TestHumanoid();
Tree TestSimpleHumanoid();
Tree TestSingleLink();
Tree TestSingleJoint();
Tree TestKukaLWR();
Tree TestDoubleJoint();

}
}
#endif