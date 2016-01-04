/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013 Istituto Italiano di Tecnologia
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

/* Author: Silvio Traversaro */


#include <iDynTree/ModelIO/symoro_par_model.hpp>
#include <iDynTree/ModelIO/symoro_par_import.hpp>

#include <iDynTree/ModelIO/impl/urdf_export.hpp>
#include <kdl/tree.hpp>

#include <iostream>

#include <cstdlib>


using namespace KDL;
using namespace std;
using namespace iDynTree;

int main(int argc, char** argv)
{
  if (argc != 3){
    std::cerr << "Usage: par2urdf robot.par robot.urdf" << std::endl;
    return -1;
  }

  symoro_par_model mdl;

  if( !parModelFromFile(argv[1],mdl) ) {cerr << "Could not parse SYMORO par robot model" << endl; return EXIT_FAILURE;}

  std::cout << "Extracted par file" << std::endl;
  std::cout << mdl.toString() << std::endl;


  Tree my_tree;
  if (!treeFromSymoroParFile(argv[1],my_tree,true))
  {cerr << "Could not generate robot model and extract kdl tree" << endl; return EXIT_FAILURE;}

  if( !treeToUrdfFile(argv[2],my_tree,"par_file_robot") )
  {cerr << "Could not export KDL::Tree to URDF file" << endl; return EXIT_FAILURE;}

  return EXIT_SUCCESS;
}


