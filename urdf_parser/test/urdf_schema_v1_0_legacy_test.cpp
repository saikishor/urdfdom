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

/// @note Link – inertial element
TEST(URDF_V1_0_LEGACY, inertial_positive_values)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <inertial>
          <mass value="5.25"/>
          <inertia ixx="0.1" ixy="0.01" ixz="0.001"
                   iyy="0.2" iyz="0.002" izz="0.3"/>
        </inertial>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->inertial);
  EXPECT_DOUBLE_EQ(5.25,  link->inertial->mass);
  EXPECT_DOUBLE_EQ(0.1,   link->inertial->ixx);
  EXPECT_DOUBLE_EQ(0.01,  link->inertial->ixy);
  EXPECT_DOUBLE_EQ(0.001, link->inertial->ixz);
  EXPECT_DOUBLE_EQ(0.2,   link->inertial->iyy);
  EXPECT_DOUBLE_EQ(0.002, link->inertial->iyz);
  EXPECT_DOUBLE_EQ(0.3,   link->inertial->izz);
}

TEST(URDF_V1_0_LEGACY, inertial_negative_mass_allowed_v1_0)
{
  // v1.0 applies no sign validation on mass.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <inertial>
          <mass value="-3.0"/>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->inertial);
  EXPECT_DOUBLE_EQ(-3.0, link->inertial->mass);
}

TEST(URDF_V1_0_LEGACY, inertial_zero_mass_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <inertial>
          <mass value="0.0"/>
          <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->inertial);
  EXPECT_DOUBLE_EQ(0.0, link->inertial->mass);
}

TEST(URDF_V1_0_LEGACY, inertial_negative_inertia_tensor_allowed_v1_0)
{
  // Negative off-diagonal terms are physically allowed; v1.0 doesn't validate.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="0.5" ixy="-0.1" ixz="-0.2"
                   iyy="0.6" iyz="-0.3" izz="0.7"/>
        </inertial>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_NE(nullptr, link);
  ASSERT_NE(nullptr, link->inertial);
  EXPECT_DOUBLE_EQ(-0.1, link->inertial->ixy);
  EXPECT_DOUBLE_EQ(-0.2, link->inertial->ixz);
  EXPECT_DOUBLE_EQ(-0.3, link->inertial->iyz);
}

TEST(URDF_V1_0_LEGACY, inertial_with_origin)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <inertial>
          <origin xyz="0.1 -0.2 0.3" rpy="0 0 1.5707963"/>
          <mass value="2.0"/>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_NE(nullptr, link->inertial);
  EXPECT_NEAR(0.1,  link->inertial->origin.position.x, 1e-9);
  EXPECT_NEAR(-0.2, link->inertial->origin.position.y, 1e-9);
  EXPECT_NEAR(0.3,  link->inertial->origin.position.z, 1e-9);
}

/// @note Geometry – all supported types in v1.0
TEST(URDF_V1_0_LEGACY, box_geometry_positive_values)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry><box size="1.0 2.0 3.0"/></geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_FALSE(link->visual_array.empty());
  EXPECT_EQ(urdf::Geometry::BOX, link->visual_array[0]->geometry->type);
  auto box = std::dynamic_pointer_cast<urdf::Box>(link->visual_array[0]->geometry);
  ASSERT_NE(nullptr, box);
  EXPECT_DOUBLE_EQ(1.0, box->dim.x);
  EXPECT_DOUBLE_EQ(2.0, box->dim.y);
  EXPECT_DOUBLE_EQ(3.0, box->dim.z);
}

TEST(URDF_V1_0_LEGACY, box_geometry_zero_dimension_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><box size="1.0 0.0 3.0"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_FALSE(model->getLink("base")->visual_array.empty());
}

TEST(URDF_V1_0_LEGACY, box_geometry_no_size_attr_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><box/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  // parseVisual fails → parseLink returns false → but model still built (link added).
  // The link exists but has no visual.
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->getLink("base")->visual_array.empty());
}

TEST(URDF_V1_0_LEGACY, sphere_geometry_positive_value)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><sphere radius="0.75"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_FALSE(link->visual_array.empty());
  auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(link->visual_array[0]->geometry);
  ASSERT_NE(nullptr, sphere);
  EXPECT_DOUBLE_EQ(0.75, sphere->radius);
}

TEST(URDF_V1_0_LEGACY, sphere_geometry_large_radius_allowed_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><sphere radius="1e6"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, sphere);
  EXPECT_DOUBLE_EQ(1e6, sphere->radius);
}

TEST(URDF_V1_0_LEGACY, sphere_geometry_negative_radius_allowed_v1_0)
{
  // v1.0 applies no sign validation on geometry dimensions.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><sphere radius="-0.5"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, sphere);
  EXPECT_DOUBLE_EQ(-0.5, sphere->radius);
}

TEST(URDF_V1_0_LEGACY, sphere_geometry_no_radius_attr_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><sphere/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->getLink("base")->visual_array.empty());
}

TEST(URDF_V1_0_LEGACY, cylinder_geometry_positive_values)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry><cylinder radius="0.4" length="1.2"/></geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, cyl);
  EXPECT_DOUBLE_EQ(0.4, cyl->radius);
  EXPECT_DOUBLE_EQ(1.2, cyl->length);
}

TEST(URDF_V1_0_LEGACY, cylinder_geometry_negative_radius_allowed_v1_0)
{
  // v1.0 applies no sign validation on geometry dimensions.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><cylinder radius="-0.4" length="1.2"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, cyl);
  EXPECT_DOUBLE_EQ(-0.4, cyl->radius);
  EXPECT_DOUBLE_EQ( 1.2, cyl->length);
}

TEST(URDF_V1_0_LEGACY, cylinder_geometry_negative_length_allowed_v1_0)
{
  // v1.0 applies no sign validation on geometry dimensions.
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><cylinder radius="0.4" length="-1.2"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, cyl);
  EXPECT_DOUBLE_EQ( 0.4, cyl->radius);
  EXPECT_DOUBLE_EQ(-1.2, cyl->length);
}

TEST(URDF_V1_0_LEGACY, cylinder_geometry_no_length_attr_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><cylinder radius="0.5"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->getLink("base")->visual_array.empty());
}

TEST(URDF_V1_0_LEGACY, cylinder_geometry_no_radius_attr_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><cylinder length="1.0"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->getLink("base")->visual_array.empty());
}

TEST(URDF_V1_0_LEGACY, mesh_geometry_with_filename)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry>
            <mesh filename="package://my_pkg/meshes/base.dae"/>
          </geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, mesh);
  EXPECT_EQ("package://my_pkg/meshes/base.dae", mesh->filename);
}

TEST(URDF_V1_0_LEGACY, mesh_geometry_default_scale_is_one)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry><mesh filename="my_mesh.stl"/></geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, mesh);
  EXPECT_DOUBLE_EQ(1.0, mesh->scale.x);
  EXPECT_DOUBLE_EQ(1.0, mesh->scale.y);
  EXPECT_DOUBLE_EQ(1.0, mesh->scale.z);
}

TEST(URDF_V1_0_LEGACY, mesh_geometry_with_explicit_scale)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry>
            <mesh filename="my_mesh.stl" scale="0.001 0.001 0.001"/>
          </geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(
    model->getLink("base")->visual_array[0]->geometry);
  ASSERT_NE(nullptr, mesh);
  EXPECT_DOUBLE_EQ(0.001, mesh->scale.x);
  EXPECT_DOUBLE_EQ(0.001, mesh->scale.y);
  EXPECT_DOUBLE_EQ(0.001, mesh->scale.z);
}

TEST(URDF_V1_0_LEGACY, mesh_geometry_no_filename_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><mesh/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->getLink("base")->visual_array.empty());
}

TEST(URDF_V1_0_LEGACY, unknown_geometry_type_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual><geometry><torus radius="1.0"/></geometry></visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->getLink("base")->visual_array.empty());
}

/// @note Visual and Collision – structure and naming
TEST(URDF_V1_0_LEGACY, visual_with_name_attr)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual name="my_visual">
          <geometry><sphere radius="0.1"/></geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ(1u, model->getLink("base")->visual_array.size());
  EXPECT_EQ("my_visual", model->getLink("base")->visual_array[0]->name);
}

TEST(URDF_V1_0_LEGACY, collision_with_name_attr)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <collision name="my_col">
          <geometry><sphere radius="0.1"/></geometry>
        </collision>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ(1u, model->getLink("base")->collision_array.size());
  EXPECT_EQ("my_col", model->getLink("base")->collision_array[0]->name);
}

TEST(URDF_V1_0_LEGACY, visual_with_origin)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <origin xyz="1.0 -2.0 3.0" rpy="0.1 0.2 0.3"/>
          <geometry><sphere radius="0.1"/></geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_FALSE(model->getLink("base")->visual_array.empty());
  const auto & origin = model->getLink("base")->visual_array[0]->origin;
  EXPECT_NEAR( 1.0, origin.position.x, 1e-9);
  EXPECT_NEAR(-2.0, origin.position.y, 1e-9);
  EXPECT_NEAR( 3.0, origin.position.z, 1e-9);
}

TEST(URDF_V1_0_LEGACY, collision_with_origin)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <collision>
          <origin xyz="-0.5 0.0 0.25" rpy="0 0 0"/>
          <geometry><box size="1 1 1"/></geometry>
        </collision>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_FALSE(model->getLink("base")->collision_array.empty());
  const auto & origin = model->getLink("base")->collision_array[0]->origin;
  EXPECT_NEAR(-0.5,  origin.position.x, 1e-9);
  EXPECT_NEAR( 0.0,  origin.position.y, 1e-9);
  EXPECT_NEAR( 0.25, origin.position.z, 1e-9);
}

TEST(URDF_V1_0_LEGACY, multiple_visuals_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual name="v1"><geometry><sphere radius="0.1"/></geometry></visual>
        <visual name="v2"><geometry><box size="1 1 1"/></geometry></visual>
        <visual name="v3">
          <geometry><cylinder radius="0.2" length="0.5"/></geometry>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_EQ(3u, link->visual_array.size());
  EXPECT_EQ(urdf::Geometry::SPHERE,   link->visual_array[0]->geometry->type);
  EXPECT_EQ(urdf::Geometry::BOX,      link->visual_array[1]->geometry->type);
  EXPECT_EQ(urdf::Geometry::CYLINDER, link->visual_array[2]->geometry->type);
  // .visual pointer must point to the first element
  EXPECT_EQ(link->visual_array[0], link->visual);
}

TEST(URDF_V1_0_LEGACY, multiple_collisions_v1_0)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <collision name="c1"><geometry><sphere radius="0.1"/></geometry></collision>
        <collision name="c2"><geometry><box size="1 1 1"/></geometry></collision>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  ASSERT_EQ(2u, link->collision_array.size());
  EXPECT_EQ(link->collision_array[0], link->collision);
}

TEST(URDF_V1_0_LEGACY, visual_and_collision_in_same_link)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry><sphere radius="0.5"/></geometry>
        </visual>
        <collision>
          <geometry><box size="1 1 1"/></geometry>
        </collision>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto link = model->getLink("base");
  EXPECT_EQ(1u, link->visual_array.size());
  EXPECT_EQ(1u, link->collision_array.size());
  EXPECT_EQ(urdf::Geometry::SPHERE, link->visual->geometry->type);
  EXPECT_EQ(urdf::Geometry::BOX,    link->collision->geometry->type);
}

/// @note Material parsing
TEST(URDF_V1_0_LEGACY, global_material_with_color)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
      <link name="base">
        <visual>
          <geometry><sphere radius="0.1"/></geometry>
          <material name="blue"/>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto mat = model->getMaterial("blue");
  ASSERT_NE(nullptr, mat);
  EXPECT_FLOAT_EQ(0.0f, static_cast<float>(mat->color.r));
  EXPECT_FLOAT_EQ(0.0f, static_cast<float>(mat->color.g));
  EXPECT_FLOAT_EQ(1.0f, static_cast<float>(mat->color.b));
  EXPECT_FLOAT_EQ(1.0f, static_cast<float>(mat->color.a));
}

TEST(URDF_V1_0_LEGACY, global_material_with_texture)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <material name="checker">
        <texture filename="package://my_pkg/textures/checker.png"/>
      </material>
      <link name="base">
        <visual>
          <geometry><sphere radius="0.1"/></geometry>
          <material name="checker"/>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto mat = model->getMaterial("checker");
  ASSERT_NE(nullptr, mat);
  EXPECT_EQ("package://my_pkg/textures/checker.png", mat->texture_filename);
}

TEST(URDF_V1_0_LEGACY, duplicate_global_material_fails)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <material name="red"><color rgba="1 0 0 1"/></material>
      <material name="red"><color rgba="0.8 0 0 1"/></material>
      <link name="base"/>
    </robot>
  )urdf";
  EXPECT_EQ(nullptr, urdf::parseURDF(urdf_str));
}

TEST(URDF_V1_0_LEGACY, inline_material_definition_in_visual)
{
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry><sphere radius="0.1"/></geometry>
          <material name="green_ish">
            <color rgba="0.1 0.9 0.1 0.5"/>
          </material>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  ASSERT_FALSE(model->getLink("base")->visual_array.empty());
  auto vis = model->getLink("base")->visual_array[0];
  ASSERT_NE(nullptr, vis->material);
  EXPECT_EQ("green_ish", vis->material->name);
  EXPECT_FLOAT_EQ(0.1f, static_cast<float>(vis->material->color.r));
  EXPECT_FLOAT_EQ(0.9f, static_cast<float>(vis->material->color.g));
  EXPECT_FLOAT_EQ(0.5f, static_cast<float>(vis->material->color.a));
}

TEST(URDF_V1_0_LEGACY, visual_color_rgba_boundary_values)
{
  // Test rgba with 0.0 and 1.0 boundary values
  std::string urdf_str = R"urdf(
    <robot name="r" version="1.0">
      <link name="base">
        <visual>
          <geometry><sphere radius="0.1"/></geometry>
          <material name="">
            <color rgba="0.0 1.0 0.0 1.0"/>
          </material>
        </visual>
      </link>
    </robot>
  )urdf";
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_str);
  ASSERT_NE(nullptr, model);
  auto vis = model->getLink("base")->visual_array[0];
  ASSERT_NE(nullptr, vis->material);
  EXPECT_FLOAT_EQ(0.0f, static_cast<float>(vis->material->color.r));
  EXPECT_FLOAT_EQ(1.0f, static_cast<float>(vis->material->color.g));
  EXPECT_FLOAT_EQ(0.0f, static_cast<float>(vis->material->color.b));
  EXPECT_FLOAT_EQ(1.0f, static_cast<float>(vis->material->color.a));
}
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  setlocale(LC_ALL, "");
  return RUN_ALL_TESTS();
}
