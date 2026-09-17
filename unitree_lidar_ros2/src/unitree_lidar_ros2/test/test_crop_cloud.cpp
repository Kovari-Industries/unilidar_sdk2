#include "crop_cloud.hpp"
#include <gtest/gtest.h>
#include <cstring>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

template<class T> void put(sensor_msgs::msg::PointCloud2 & cloud, size_t offset, T value)
{
  std::memcpy(cloud.data.data() + offset, &value, sizeof(value));
}

TEST(CropCloud, PreservesTimingRingAndOpaqueFieldsAfterTransform)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  cloud.header.stamp.sec = 123;
  cloud.header.stamp.nanosec = 123456789;
  cloud.height = 1;
  cloud.width = 3;
  cloud.point_step = 32;
  cloud.row_step = 96;
  cloud.is_dense = true;
  const char * names[] = {"x", "y", "z", "intensity", "ring", "time", "opaque"};
  const unsigned offsets[] = {0, 4, 8, 16, 20, 24, 28};
  for (size_t i = 0; i < 7; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i]; f.offset = offsets[i]; f.count = 1;
    f.datatype = i == 4 ? 4 : (i == 6 ? 6 : 7);
    cloud.fields.push_back(f);
  }
  cloud.data.resize(96);
  for (size_t i = 0; i < 3; ++i) {
    put(cloud, i * 32, float(i) - 1);
    put(cloud, i * 32 + 16, 42.f + i);
    put(cloud, i * 32 + 20, uint16_t(7 + i));
    put(cloud, i * 32 + 24, .001f * i);
    put(cloud, i * 32 + 28, uint32_t(0xdead0000 + i));
  }
  geometry_msgs::msg::TransformStamped tf;
  tf.header = cloud.header; tf.header.frame_id = "base_link";
  tf.transform.rotation.w = 1;
  tf.transform.translation.z = .3;
  sensor_msgs::msg::PointCloud2 transformed;
  tf2::doTransform(cloud, transformed, tf);
  const auto output = crop_cloud_preserving_fields(
    transformed, Eigen::Vector4f(-.5, -.5, -.5, 1),
    Eigen::Vector4f(.5, .5, .5, 1), true);
  EXPECT_EQ(output.header, transformed.header);
  EXPECT_EQ(output.fields, cloud.fields);
  EXPECT_EQ(output.point_step, cloud.point_step);
  ASSERT_EQ(output.width * output.height, 2u);
  ASSERT_EQ(output.data.size(), 64u);
  EXPECT_EQ(std::memcmp(output.data.data(), transformed.data.data(), 32), 0);
  EXPECT_EQ(std::memcmp(output.data.data() + 32, transformed.data.data() + 64, 32), 0);
  const auto inside = crop_cloud_preserving_fields(
    transformed, Eigen::Vector4f(-.5, -.5, -.5, 1),
    Eigen::Vector4f(.5, .5, .5, 1), false);
  ASSERT_EQ(inside.width * inside.height, 1u);
  EXPECT_EQ(std::memcmp(inside.data.data(), transformed.data.data() + 32, 32), 0);
}
