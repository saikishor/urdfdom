// Copyright 2026 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <string>

#include "urdf_model/joint.h"
#include "urdf_model/link.h"
#include "urdf_model/pose.h"
#include "urdf_parser/urdf_parser.h"

/// @note Model-level structure and validation
TEST(URDF_V1_0_LEGACY, no_version_attr_defaults_to_v1_0)
{
  // When version is omitted the parser defaults to 1.0.
  std::string urdf_str = R"urdf(
    <robot name="no_version">
      <link name="base"/>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("no_version", model->name_);
  EXPECT_EQ(1u, model->links_.size());
}

TEST(URDF_V1_0_LEGACY, single_link_robot_is_valid)
{
  std::string urdf_str = R"urdf(
    <robot name="single" version="1.0">
      <link name="only_link"/>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(1u, model->links_.size());
  ASSERT_NE(nullptr, model->root_link_);
  EXPECT_EQ("only_link", model->root_link_->name);
}

TEST(URDF_V1_0_LEGACY, invalid_xml_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF("<robot name=\"bad\" version=\"1.0\"");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_V1_0_LEGACY, no_robot_element_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(
    "<not_a_robot name=\"r\"><link name=\"l\"/></not_a_robot>");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_V1_0_LEGACY, robot_without_name_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(
    "<robot version=\"1.0\"><link name=\"l\"/></robot>");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_V1_0_LEGACY, no_links_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(
    "<robot name=\"empty\" version=\"1.0\"></robot>");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_V1_0_LEGACY, unsupported_version_too_high_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(
    "<robot name=\"r\" version=\"2.0\"><link name=\"l\"/></robot>");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_V1_0_LEGACY, unsupported_version_too_low_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(
    "<robot name=\"r\" version=\"0.9\"><link name=\"l\"/></robot>");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_V1_0_LEGACY, malformed_version_string_fails)
{
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(
    "<robot name=\"r\" version=\"one.zero\"><link name=\"l\"/></robot>");
  EXPECT_EQ(nullptr, model);
}

TEST(URDF_V1_0_LEGACY, duplicate_link_name_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="dup" version="1.0">
      <link name="base"/>
      <link name="base"/>
    </robot>
  )urdf";
  EXPECT_EQ(nullptr, urdf::parseURDF(urdf_str));
}

TEST(URDF_V1_0_LEGACY, duplicate_joint_name_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="dup" version="1.0">
      <link name="l1"/>
      <link name="l2"/>
      <link name="l3"/>
      <joint name="j1" type="fixed">
        <parent link="l1"/><child link="l2"/>
      </joint>
      <joint name="j1" type="fixed">
        <parent link="l1"/><child link="l3"/>
      </joint>
    </robot>
  )urdf";
  EXPECT_EQ(nullptr, urdf::parseURDF(urdf_str));
}

TEST(URDF_V1_0_LEGACY, joint_referencing_unknown_parent_link_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="child"/>
      <joint name="j1" type="fixed">
        <parent link="nonexistent"/>
        <child link="child"/>
      </joint>
    </robot>
  )urdf";
  EXPECT_EQ(nullptr, urdf::parseURDF(urdf_str));
}

TEST(URDF_V1_0_LEGACY, joint_referencing_unknown_child_link_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base"/>
      <joint name="j1" type="fixed">
        <parent link="base"/>
        <child link="nonexistent"/>
      </joint>
    </robot>
  )urdf";
  EXPECT_EQ(nullptr, urdf::parseURDF(urdf_str));
}

TEST(URDF_V1_0_LEGACY, two_root_links_fails)
{
  // Two disconnected links with no joint → two root links → initRoot fails.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="root1"/>
      <link name="root2"/>
    </robot>
  )urdf";
  EXPECT_EQ(nullptr, urdf::parseURDF(urdf_str));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
