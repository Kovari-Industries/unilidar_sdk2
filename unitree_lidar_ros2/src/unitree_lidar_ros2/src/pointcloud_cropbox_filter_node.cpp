/**
 * Point Cloud CropBox Filter
 *
 * Removes points inside a 3D bounding box to filter out the robot body
 * from LiDAR scans. Box dimensions default to the robot footprint from
 * nav2_params.
 *
 * If target_frame is set, the cloud is transformed into that frame before
 * filtering, and the output is published in the target frame.
 */

#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class PointCloudCropBoxFilter : public rclcpp::Node
{
public:
  PointCloudCropBoxFilter()
  : Node("pointcloud_cropbox_filter")
  {
    // Crop box dimensions (defaults from nav2_params robot footprint)
    declare_parameter("crop_min_x", -0.70);
    declare_parameter("crop_max_x", 0.09);
    declare_parameter("crop_min_y", -0.35);
    declare_parameter("crop_max_y", 0.34);
    declare_parameter("crop_min_z", -0.50);
    declare_parameter("crop_max_z", 0.50);
    declare_parameter("negative", true);
    declare_parameter("input_topic", std::string("unilidar/cloud"));
    declare_parameter("output_topic", std::string("unilidar/cloud_filtered"));
    declare_parameter("target_frame", std::string(""));

    load_crop_params();
    negative_ = get_parameter("negative").as_bool();
    auto input_topic = get_parameter("input_topic").as_string();
    auto output_topic = get_parameter("output_topic").as_string();
    target_frame_ = get_parameter("target_frame").as_string();

    if (!target_frame_.empty()) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      RCLCPP_INFO(get_logger(), "Will transform cloud to frame: %s", target_frame_.c_str());
    }

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudCropBoxFilter::cloud_callback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

    // Dynamic parameter updates
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&PointCloudCropBoxFilter::param_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "CropBox filter: [%.2f, %.2f] x [%.2f, %.2f] x [%.2f, %.2f] (negative=%s)",
      crop_min_x_, crop_max_x_, crop_min_y_, crop_max_y_, crop_min_z_, crop_max_z_,
      negative_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  %s -> %s", input_topic.c_str(), output_topic.c_str());
  }

private:
  void load_crop_params()
  {
    crop_min_x_ = get_parameter("crop_min_x").as_double();
    crop_max_x_ = get_parameter("crop_max_x").as_double();
    crop_min_y_ = get_parameter("crop_min_y").as_double();
    crop_max_y_ = get_parameter("crop_max_y").as_double();
    crop_min_z_ = get_parameter("crop_min_z").as_double();
    crop_max_z_ = get_parameter("crop_max_z").as_double();
  }

  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      if (p.get_name() == "crop_min_x") crop_min_x_ = p.as_double();
      else if (p.get_name() == "crop_max_x") crop_max_x_ = p.as_double();
      else if (p.get_name() == "crop_min_y") crop_min_y_ = p.as_double();
      else if (p.get_name() == "crop_max_y") crop_max_y_ = p.as_double();
      else if (p.get_name() == "crop_min_z") crop_min_z_ = p.as_double();
      else if (p.get_name() == "crop_max_z") crop_max_z_ = p.as_double();
      else if (p.get_name() == "negative") negative_ = p.as_bool();
    }
    RCLCPP_INFO(get_logger(),
      "CropBox updated: [%.2f, %.2f] x [%.2f, %.2f] x [%.2f, %.2f] (negative=%s)",
      crop_min_x_, crop_max_x_, crop_min_y_, crop_max_y_, crop_min_z_, crop_max_z_,
      negative_ ? "true" : "false");
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 cloud_in;

    if (!target_frame_.empty() && tf_buffer_) {
      try {
        cloud_in = tf_buffer_->transform(*msg, target_frame_, tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TF transform failed: %s", ex.what());
        return;
      }
    } else {
      cloud_in = *msg;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(cloud_in, *cloud);

    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(crop_min_x_, crop_min_y_, crop_min_z_, 1.0f));
    crop.setMax(Eigen::Vector4f(crop_max_x_, crop_max_y_, crop_max_z_, 1.0f));
    crop.setNegative(negative_);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    crop.filter(*filtered);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.header = cloud_in.header;

    pub_->publish(output);
  }

  double crop_min_x_, crop_max_x_;
  double crop_min_y_, crop_max_y_;
  double crop_min_z_, crop_max_z_;
  bool negative_;
  std::string target_frame_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudCropBoxFilter>());
  rclcpp::shutdown();
  return 0;
}
