#pragma once

namespace livox_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace livox_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
                                (float, x, x)
                                (float, y, y)
                                (float, z, z)
                                (float, intensity, intensity)
                                (std::uint8_t, tag, tag)
                                (std::uint8_t, line, line)
                                (double, timestamp, timestamp)
)
// clang-format on