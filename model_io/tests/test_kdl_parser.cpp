/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
