/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// SDK
#include "unitree_lidar_sdk_pcl.h"


using std::placeholders::_1;

class UnitreeLidarSDKNode : public rclcpp::Node
{
public:
    explicit UnitreeLidarSDKNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~UnitreeLidarSDKNode() {};

    void timer_callback();

protected:
    // ROS
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // Unitree Lidar Reader
    UnitreeLidarReader *lsdk_;

    // Config params
    int work_mode_;
    int initialize_type_;
    int local_port_;
    std::string local_ip_;
    int lidar_port_;
    std::string lidar_ip_;
    std::string serial_port_;
    int baudrate_;
    
    int cloud_scan_num_;
    bool use_system_timestamp_;
    double range_min_;
    double range_max_;

    std::string cloud_frame_;
    std::string cloud_topic_;

    std::string imu_frame_;
    std::string imu_topic_;
};

///////////////////////////////////////////////////////////////////

UnitreeLidarSDKNode::UnitreeLidarSDKNode(const rclcpp::NodeOptions &options)
    : Node("unitre_lidar_sdk_node", options)
{
    // load config parameters
    declare_parameter<int>("initialize_type", 2);
    declare_parameter<int>("work_mode", 0);
    declare_parameter<bool>("use_system_timestamp", true);
    declare_parameter<double>("range_min", 0);
    declare_parameter<double>("range_max", 50);
    declare_parameter<int>("cloud_scan_num", 18);

    declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 4000000);

    declare_parameter<int>("lidar_port", 6101);
    declare_parameter<std::string>("lidar_ip", "192.168.1.2");
    declare_parameter<int>("local_port", 6201);
    declare_parameter<std::string>("local_ip", "192.168.1.62");

    declare_parameter<std::string>("cloud_frame", "unilidar_lidar");
    declare_parameter<std::string>("cloud_topic", "unilidar/cloud");

    declare_parameter<std::string>("imu_frame", "unilidar_imu");
    declare_parameter<std::string>("imu_topic", "unilidar/imu");

    work_mode_ = get_parameter("work_mode").as_int();
    initialize_type_ = get_parameter("initialize_type").as_int();

    serial_port_ = get_parameter("serial_port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();

    lidar_port_ = get_parameter("lidar_port").as_int();
    lidar_ip_ = get_parameter("lidar_ip").as_string();
    local_port_ = get_parameter("local_port").as_int();
    local_ip_ = get_parameter("local_ip").as_string();

    cloud_scan_num_ = get_parameter("cloud_scan_num").as_int();
    use_system_timestamp_ = get_parameter("use_system_timestamp").as_bool();
    range_max_ = get_parameter("range_max").as_double();
    range_min_ = get_parameter("range_min").as_double();

    cloud_frame_ = get_parameter("cloud_frame").as_string();
    cloud_topic_ = get_parameter("cloud_topic").as_string();
    
    imu_frame_ = get_parameter("imu_frame").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();

    // Initialize UnitreeLidarReader
    lsdk_ = createUnitreeLidarReader();

    std::cout << "initialize_type_ = " << initialize_type_ << std::endl;

    if (initialize_type_ == 1)
    {
        lsdk_->initializeSerial(serial_port_, baudrate_,
                                cloud_scan_num_, use_system_timestamp_, range_min_, range_max_);
    }
    else if (initialize_type_ == 2)
    {
        lsdk_->initializeUDP(lidar_port_, lidar_ip_, local_port_, local_ip_,
                             cloud_scan_num_, use_system_timestamp_, range_min_, range_max_);
    }
    else
    {
        std::cout << "initialize_type is not right! exit now ...\n";
        exit(0);
    }

    lsdk_->setLidarWorkMode(work_mode_);

    // ROS2
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&UnitreeLidarSDKNode::timer_callback, this));
}

void UnitreeLidarSDKNode::timer_callback()
{
    int result = lsdk_->runParse();
    static pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    if (result == LIDAR_IMU_DATA_PACKET_TYPE)
    {
        LidarImuData imu;
        if (lsdk_->getImuData(imu))
        {
            // Optional: You can still publish the IMU topic data if needed
            rclcpp::Time timestamp(imu.info.stamp.sec, imu.info.stamp.nsec);
            sensor_msgs::msg::Imu imuMsg;
            imuMsg.header.frame_id = imu_frame_;
            imuMsg.header.stamp = timestamp;
            // ... (fill in orientation/velocity/acceleration as before)
            pub_imu_->publish(imuMsg);

            // REMOVED: All broadcaster_->sendTransform calls are gone from here.
            // This prevents the SDK from creating the "randomly angled" IMU frames.
        }
    }
    else if (result == LIDAR_POINT_DATA_PACKET_TYPE)
    {
        PointCloudUnitree cloud;
        if (lsdk_->getPointCloud(cloud))
        {
            transformUnitreeCloudToPCL(cloud, cloudOut);

            rclcpp::Time timestamp(
                static_cast<int32_t>(cloud.stamp),
                static_cast<uint32_t>((cloud.stamp - static_cast<int32_t>(cloud.stamp)) * 1e9));

            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloudOut, cloud_msg);

            // This remains "unilidar_lidar" (or whatever cloud_frame_ is set to)
            // Your URDF will now provide the static transform for this frame.
            cloud_msg.header.frame_id = cloud_frame_;
            cloud_msg.header.stamp = timestamp;

            pub_cloud_->publish(cloud_msg);
        }
    }
    else if (result == LIDAR_2D_POINT_DATA_PACKET_TYPE)
    {
        // Only handle 2D packets when in 2D measurement mode (work_mode bit 1 = 1).
        // In 3D mode (workmode 0), the firmware may still emit type-103 packets, but
        // parseFromPacketPointCloud2D sets x=0 for every point (no horizontal rotation),
        // producing flat planes that corrupt the 3D cloud topic. The 3D cloud is built
        // exclusively from LIDAR_POINT_DATA_PACKET_TYPE packets via getPointCloud() above.
        if (!(work_mode_ & 0x02))
        {
            return;
        }

        PointCloudUnitree cloud;
        parseFromPacketPointCloud2D(cloud, lsdk_->getLidar2DPointDataPacket(),
                                    use_system_timestamp_, range_min_, range_max_);

        transformUnitreeCloudToPCL(cloud, cloudOut);

        rclcpp::Time timestamp(
            static_cast<int32_t>(cloud.stamp),
            static_cast<uint32_t>((cloud.stamp - static_cast<int32_t>(cloud.stamp)) * 1e9));

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloudOut, cloud_msg);
        cloud_msg.header.frame_id = cloud_frame_;
        cloud_msg.header.stamp = timestamp;

        pub_cloud_->publish(cloud_msg);
    }
}