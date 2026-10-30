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
#include <cmath>
#include <deque>
#include <limits>
#include <utility>

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

/*
 * Maps the lidar's hardware clock to host (ROS) time.
 *
 * The L2 (firmware 2.8.11.1) sets its clock near host time on connection, but
 * the clock then advances at about half speed (0.501 s per real second), so
 * hardware stamps fall behind by ~0.5 s every second and syncLidarTimeStamp()
 * only resets the offset. The stamps are still precise relative to each other
 * (point-packet stamps sit within ~40 us of a straight line), so fit host
 * arrival time against hardware time over a sliding window and take the lower
 * envelope: packets arrive late by network and polling delay but never early.
 * On a 60 s capture this cut stamp-interval jitter from 395 us to 0.7 us (IMU)
 * and from 73 us to 18 us (point packets).
 * The result keeps the hardware's relative precision, absorbs the rate error
 * and any drift, and carries only the constant minimum transport latency.
 */
class LidarClockMap
{
public:
    void observe(double hw, double arrival)
    {
        if (!win_.empty() && std::abs(arrival - map_unchecked(hw)) > kResetError)
            reset();
        if (win_.empty()) { hw_ref_ = hw; arrival_ref_ = arrival; }
        win_.emplace_back(hw - hw_ref_, arrival - arrival_ref_);
        while (win_.back().second - win_.front().second > kWindow)
            win_.pop_front();
        if (++since_fit_ >= kRefitEvery && win_.back().second - win_.front().second >= kMinSpan)
            fit();
    }
    bool ready() const { return fitted_; }
    double map(double hw) const { return map_unchecked(hw); }
    double rate() const { return rate_; }
    double spread() const { return spread_; }

private:
    static constexpr double kWindow = 10.0;     // s of host time
    static constexpr double kMinSpan = 2.0;     // s before the first fit
    static constexpr double kResetError = 0.2;  // s; a clock step or long dropout
    static constexpr int kRefitEvery = 50;

    double map_unchecked(double hw) const
    {
        return fitted_ ? arrival_ref_ + offset_ + rate_ * (hw - hw_ref_) : arrival_ref_ + (hw - hw_ref_) * 2.0;
    }
    void reset() { win_.clear(); fitted_ = false; since_fit_ = 0; }
    void fit()
    {
        since_fit_ = 0;
        double mx = 0, my = 0;
        for (auto &p : win_) { mx += p.first; my += p.second; }
        mx /= win_.size(); my /= win_.size();
        double num = 0, den = 0;
        for (auto &p : win_) { num += (p.first - mx) * (p.second - my); den += (p.first - mx) * (p.first - mx); }
        if (den <= 0) return;
        double b = num / den, lo = std::numeric_limits<double>::max(), hi = -lo;
        for (auto &p : win_) { double r = p.second - b * p.first; lo = std::min(lo, r); hi = std::max(hi, r); }
        rate_ = b; offset_ = lo; spread_ = hi - lo; fitted_ = true;
    }

    std::deque<std::pair<double, double>> win_;
    double hw_ref_ = 0, arrival_ref_ = 0, rate_ = 2.0, offset_ = 0, spread_ = 0;
    bool fitted_ = false;
    int since_fit_ = 0;
};

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
    bool hardware_clock_map_;
    LidarClockMap clock_map_;
    rclcpp::Time last_clock_log_{0, 0, RCL_ROS_TIME};
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
    // Map hardware stamps to host time (see LidarClockMap); overrides use_system_timestamp.
    declare_parameter<bool>("hardware_clock_map", true);
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
    hardware_clock_map_ = get_parameter("hardware_clock_map").as_bool();
    if (hardware_clock_map_)
        use_system_timestamp_ = false;  // the SDK must hand over hardware stamps
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

static rclcpp::Time seconds_to_ros_time(double t)
{
    int32_t sec = static_cast<int32_t>(std::floor(t));
    int64_t nsec = std::llround((t - sec) * 1e9);
    if (nsec >= 1000000000) { sec += 1; nsec -= 1000000000; }
    return rclcpp::Time(sec, static_cast<uint32_t>(nsec), RCL_ROS_TIME);
}

void UnitreeLidarSDKNode::timer_callback()
{
    int result = lsdk_->runParse();
    static pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    const rclcpp::Time arrival_time = this->now();
    const double arrival = arrival_time.seconds();

    // Host time for a hardware stamp: mapped once the fit is ready, arrival time before that.
    auto host_stamp = [&](double hw) {
        if (!hardware_clock_map_)
            return seconds_to_ros_time(hw);
        return clock_map_.ready() ? seconds_to_ros_time(clock_map_.map(hw)) : arrival_time;
    };

    if (hardware_clock_map_ && (result == LIDAR_POINT_DATA_PACKET_TYPE))
    {
        // Both packet types train the map: point packets reach the host later than IMU
        // packets, so the lower envelope needs the IMU packets to stay causal.
        const auto &packet = lsdk_->getLidarPointDataPacket();
        clock_map_.observe(packet.data.info.stamp.sec + packet.data.info.stamp.nsec * 1e-9, arrival);
        if (clock_map_.ready() && (arrival_time - last_clock_log_).seconds() > 60.0)
        {
            last_clock_log_ = arrival_time;
            RCLCPP_INFO(get_logger(), "lidar clock map: %.6f host s per lidar s, arrival spread %.3f ms",
                        clock_map_.rate(), clock_map_.spread() * 1e3);
        }
    }

    if (result == LIDAR_IMU_DATA_PACKET_TYPE)
    {
        LidarImuData imu;
        if (lsdk_->getImuData(imu))
        {
            const double imu_hw = imu.info.stamp.sec + imu.info.stamp.nsec * 1e-9;
            if (hardware_clock_map_)
                clock_map_.observe(imu_hw, arrival);
            rclcpp::Time timestamp = host_stamp(imu_hw);
            // Without the clock map, use_system_timestamp puts the IMU on the
            // cloud's clock: host receipt time.
            if (!hardware_clock_map_ && use_system_timestamp_)
            {
                TimeStamp now;
                getSystemTimeStamp(now);
                timestamp = rclcpp::Time(now.sec, now.nsec);
            }
            sensor_msgs::msg::Imu imuMsg;
            imuMsg.header.frame_id = imu_frame_;
            imuMsg.header.stamp = timestamp;
            // SDK ordering is w,x,y,z; ROS stores x,y,z,w. Rates and
            // acceleration use the SDK's SI units, as in the upstream ROS driver.
            imuMsg.orientation.w = imu.quaternion[0];
            imuMsg.orientation.x = imu.quaternion[1];
            imuMsg.orientation.y = imu.quaternion[2];
            imuMsg.orientation.z = imu.quaternion[3];
            imuMsg.angular_velocity.x = imu.angular_velocity[0];
            imuMsg.angular_velocity.y = imu.angular_velocity[1];
            imuMsg.angular_velocity.z = imu.angular_velocity[2];
            imuMsg.linear_acceleration.x = imu.linear_acceleration[0];
            imuMsg.linear_acceleration.y = imu.linear_acceleration[1];
            imuMsg.linear_acceleration.z = imu.linear_acceleration[2];
            // Zero covariance means unknown, not perfect confidence. Do not
            // publish an IMU TF that conflicts with the robot's fixed mount.
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

            rclcpp::Time timestamp = host_stamp(cloud.stamp);

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

        rclcpp::Time timestamp = host_stamp(cloud.stamp);

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloudOut, cloud_msg);
        cloud_msg.header.frame_id = cloud_frame_;
        cloud_msg.header.stamp = timestamp;

        pub_cloud_->publish(cloud_msg);
    }
}