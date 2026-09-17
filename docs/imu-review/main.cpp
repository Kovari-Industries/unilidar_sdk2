#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: imu_review BAG [BAG...]\n";
    return 2;
  }
  for (int k = 1; k < argc; k++) {
    rosbag2_cpp::Reader r;
    r.open(argv[k]);
    rosbag2_storage::StorageFilter f;
    f.topics = {"/unilidar/imu"};
    r.set_filter(f);
    rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
    size_t n = 0;
    double total[2] = {}, tilt[2] = {}, max_tilt[2] = {}, accel[3] = {};
    while (r.has_next()) {
      auto b = r.read_next();
      rclcpp::SerializedMessage s(*b->serialized_data);
      sensor_msgs::msg::Imu m;
      serializer.deserialize_message(&s, &m);
      double a[3] = {m.linear_acceleration.x, m.linear_acceleration.y,
                     m.linear_acceleration.z};
      double an = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
      double qnorm = m.orientation.w * m.orientation.w +
                     m.orientation.x * m.orientation.x +
                     m.orientation.y * m.orientation.y +
                     m.orientation.z * m.orientation.z;
      if (!std::isfinite(an) || an < 1e-6 || !std::isfinite(qnorm) ||
          qnorm < 1e-12) {
        std::cerr << "Invalid IMU sample\n";
        return 1;
      }
      if (!n)
        std::cout << argv[k] << " first ROS xyzw=" << m.orientation.x << ","
                  << m.orientation.y << "," << m.orientation.z << ","
                  << m.orientation.w << " accel=" << a[0] << "," << a[1] << ","
                  << a[2] << "\n";
      // Existing b3db766 mapping: ROS xyzw = SDK indices 1,2,3,0.
      double q[2][4] = {
          {m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z},
          {m.orientation.z, m.orientation.w, m.orientation.x, m.orientation.y}};
      for (int h = 0; h < 2; h++) {
        double w = q[h][0], x = q[h][1], y = q[h][2], z = q[h][3];
        double norm = w * w + x * x + y * y + z * z;
        double g[3] = {2 * (x * z - w * y) / norm, 2 * (y * z + w * x) / norm,
                       1 - 2 * (x * x + y * y) / norm};
        double dot = (g[0] * a[0] + g[1] * a[1] + g[2] * a[2]) / an;
        total[h] += std::acos(std::clamp(dot, -1., 1.)) * 180 / M_PI;
        double angle = std::acos(std::clamp(g[2], -1., 1.)) * 180 / M_PI;
        tilt[h] += angle;
        max_tilt[h] = std::max(max_tilt[h], angle);
      }
      for (int j = 0; j < 3; j++)
        accel[j] += a[j];
      n++;
    }
    if (!n) {
      std::cerr << "No IMU samples\n";
      return 1;
    }
    std::cout << std::setprecision(8) << "samples=" << n
              << " mean_accel=" << accel[0] / n << "," << accel[1] / n << ","
              << accel[2] / n << "\n";
    for (int h = 0; h < 2; h++)
      std::cout << (h ? "SDK xyzw" : "SDK wxyz")
                << " gravity_error_deg=" << total[h] / n
                << " upright_tilt_deg=" << tilt[h] / n
                << " max_tilt_deg=" << max_tilt[h] << "\n";
  }
}
