/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "kdl_urdf/symoro_par/symoro_par_model.hpp"
#include "kdl_urdf/symoro_par/symoro_par_import.hpp"


using namespace KDL;
using namespace std;
using namespace symoro_par;

void printLink(const SegmentMap::const_iterator& link, const std::string& prefix)
{
  cout << prefix << "- Segment " << link->second.segment.getName() << " has " << link->second.children.size() << " children" << endl;
  for (unsigned int i=0; i<link->second.children.size(); i++)
    printLink(link->second.children[i], prefix + "  ");
}



int main(int argc, char** argv)
{
  if (argc < 2){
    std::cerr << "Expect .par file to parse" << std::endl;
    return -1;
  }
  
  symoro_par_model mdl;
  
  if( !parModelFromFile(argv[1],mdl) ) {cerr << "Could not parse SyMoRo par robot model" << endl; return false;}
  
  std::cout << "Extracted par file" << std::endl;
  std::cout << mdl.toString() << std::endl;

  
  Tree my_tree;
  if (!symoro_par::treeFromFile(argv[1],my_tree)) 
  {cerr << "Could not generate robot model and extract kdl tree" << endl; return false;}

  // walk through tree
  cout << " ======================================" << endl;
  cout << " Tree has " << my_tree.getNrOfSegments() << " link(s) and a root link" << endl;
  cout << " ======================================" << endl;
  SegmentMap::const_iterator root = my_tree.getRootSegment();
  printLink(root, "");
 
}


