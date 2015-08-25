/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Istituto Italiano di Tecnologia
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author:  Silvio Traversaro */

#ifndef SYMORO_PAR_IMPORT_SERIALIZATION_H
#define SYMORO_PAR_IMPORT_SERIALIZATION_H

#include <string>

#include "symoro_par_import.hpp"
#include "symoro_par_model.hpp"

namespace KDL {
    class Tree;

    namespace CoDyCo {
        class TreeSerialization;
    }
}

namespace iDynTree
{

/** Constructs a KDL::CoDyCo tree serialization from a .par file, given the file name
 *  The .par file is produced by the Symoro+ software
 * \param file The filename from where to read the .par file
 * \param serialization The resulting KDL::CoDyCo TreeSerialization
 * \param consider_root_link_inertia optional (default true) if true parse the first link
 *                  of the robot model as a real link, and introduces a dummy link connected to it
 *                 with a fixed joint to overcome the the fact that KDL does not support inertia in the first link
 * returns true on success, false on failure
 */
bool treeSerializationFromSymoroParFile(const std::string& parfile_name, KDL::CoDyCo::TreeSerialization& serialization,  const bool consider_root_link_inertia=true);

/** Constructs a KDL tree from a string of the contents of the par file
 * \param xml A string containting the Symoro+ par description of the robot
 * \param serialization The resulting KDL::CoDyCo TreeSerialization
 * \param consider_root_link_inertia optional (default true) if true parse the first link
 *                  of the robot model as a real link, and introduces a dummy link connected to it
 *                 with a fixed joint to overcome the the fact that KDL does not support inertia in the first link
 * returns true on success, false on failure
 */
bool treeSerializationFromSymoroParString(const std::string& parfile_content, KDL::CoDyCo::TreeSerialization& serialization, const bool consider_root_link_inertia=true);

/** Constructs a KDL tree from a structure representation of the contents of the par file
 * \param par_model A symoro_par_model object containing the Symoro+ par description of the robot
 * \param serialization The resulting KDL::CoDyCo TreeSerialization
 * \param consider_root_link_inertia optional (default true) if true parse the first link
 *                  of the robot model as a real link, and introduces a dummy link connected to it
 *                 with a fixed joint to overcome the the fact that KDL does not support inertia in the first link
 * returns true on success, false on failure
 */
bool treeSerializationFromParModel(const symoro_par_model & par_model, KDL::CoDyCo::TreeSerialization& serialization, const bool consider_root_link_inertia=true);

}

#endif
