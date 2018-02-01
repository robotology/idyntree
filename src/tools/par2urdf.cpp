/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

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


