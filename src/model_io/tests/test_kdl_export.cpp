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

#include <string>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "kdl_parser/kdl_parser.hpp"

using namespace kdl_parser;

int g_argc;
char** g_argv;

class TestParser : public testing::Test
{
public:
  KDL::Tree my_tree;

protected:
  /// constructor
  TestParser()
  {
  }


  /// Destructor
  ~TestParser()
  {
  }
};




TEST_F(TestParser, test)
{
  for (int i=1; i<g_argc-2; i++){
    ROS_ERROR("Testing file %s", g_argv[i]);
    ASSERT_FALSE(treeFromFile(g_argv[i], my_tree));
  }

  ASSERT_TRUE(treeFromFile(g_argv[g_argc-1], my_tree));
  ASSERT_EQ(my_tree.getNrOfJoints(), (unsigned int)44);
  ASSERT_EQ(my_tree.getNrOfSegments(), (unsigned int)81);
  ASSERT_TRUE(my_tree.getSegment("base_footprint") == my_tree.getRootSegment());
  ASSERT_EQ(my_tree.getRootSegment()->second.children.size(), (unsigned int)1);
  ASSERT_TRUE(my_tree.getSegment("base_link")->second.parent == my_tree.getRootSegment());
  ASSERT_EQ(my_tree.getSegment("base_link")->second.segment.getInertia().getMass(), 116.0);
  ASSERT_NEAR(my_tree.getSegment("base_link")->second.segment.getInertia().getRotationalInertia().data[0], 15.6107, 0.001);
  SUCCEED();
}




int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_kdl_parser");
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
