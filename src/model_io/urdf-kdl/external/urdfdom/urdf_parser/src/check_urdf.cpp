/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
/* Author: Wim Meeussen */

#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>

using namespace urdf;

void printTree(ConstLinkPtr link,int level = 0)
{
  level+=2;
  int count = 0;
  for (std::vector<LinkPtr >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
  {
    if (*child)
    {
      for(int j=0;j<level;j++) std::cout << "  "; //indent
      std::cout << "child(" << (count++)+1 << "):  " << (*child)->name  << std::endl;
      // first grandchild
      printTree(*child,level);
    }
    else
    {
      for(int j=0;j<level;j++) std::cout << " "; //indent
      std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
    }
  }

}


int main(int argc, char** argv)
{
  if (argc < 2){
    std::cerr << "Expect URDF xml file to parse" << std::endl;
    return -1;
  }

  std::string xml_string;
  std::fstream xml_file(argv[1], std::fstream::in);
  while ( xml_file.good() )
  {
    std::string line;
    std::getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();

  ModelInterfacePtr robot = parseURDF(xml_string);
  if (!robot){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }
  std::cout << "robot name is: " << robot->getName() << std::endl;

  // get info from parser
  std::cout << "---------- Successfully Parsed XML ---------------" << std::endl;
  // get root link
  ConstLinkPtr root_link=robot->getRoot();
  if (!root_link) return -1;

  std::cout << "root Link: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << std::endl;


  // print entire tree
  printTree(root_link);
  return 0;
}

