#pragma once

#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

inline sensor_msgs::msg::PointCloud2 crop_cloud_preserving_fields(
  const sensor_msgs::msg::PointCloud2 & input,
  const Eigen::Vector4f & minimum, const Eigen::Vector4f & maximum, bool negative)
{
  auto cloud = std::make_shared<pcl::PCLPointCloud2>();
  pcl_conversions::toPCL(input, *cloud);
  pcl::CropBox<pcl::PCLPointCloud2> crop;
  crop.setInputCloud(cloud);
  crop.setMin(minimum);
  crop.setMax(maximum);
  crop.setNegative(negative);
  pcl::PCLPointCloud2 filtered;
  crop.filter(filtered);
  sensor_msgs::msg::PointCloud2 output;
  pcl_conversions::fromPCL(filtered, output);
  output.header = input.header;
  return output;
}
