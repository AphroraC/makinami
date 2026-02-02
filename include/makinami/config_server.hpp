#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <map>
#include <string>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace makinami {

struct OutputConfig {
  std::string topic{};
  std::string frame_id{};
};

struct LidarConfig {
  std::string topic{};
  Eigen::Vector3d translation{};
  Eigen::Quaterniond rotation{};
  Eigen::Matrix4d extrinsic_matrix{};
};

class ConfigServer {
 public:
  ConfigServer() = default;
  ~ConfigServer() = default;
  void initializeParameters(const std::string& config_file) {
    std::ifstream file(config_file);
    json data = json::parse(file);
    try {
      num_threads = data["num_threads"];
      sync_queue_size = data["sync_queue_size"];
      sync_tolerance_nanosecond = data["sync_tolerance_nanosecond"];
      sync_policy_approx = data["sync_policy_approx"];
      output.topic = data["output_params"]["topic"];
      output.frame_id = data["output_params"]["frame_id"];

      for (const auto& lidar : data["lidar_params"]) {
        std::string name = lidar["name"];
        LidarConfig param;
        param.topic = lidar["topic"];
        param.translation =
            Eigen::Vector3d(lidar["translation"][0], lidar["translation"][1],
                            lidar["translation"][2]);
        param.rotation =
            Eigen::Quaterniond(lidar["rotation"][3], lidar["rotation"][0],
                               lidar["rotation"][1], lidar["rotation"][2]);
        param.extrinsic_matrix = Eigen::Matrix4d::Identity();
        param.extrinsic_matrix.block<3, 1>(0, 3) = param.translation;
        param.extrinsic_matrix.block<3, 3>(0, 0) =
            param.rotation.toRotationMatrix();
        lidars[name] = param;
      }
    } catch (...) {
      throw std::runtime_error("Error parsing configuration file.");
    }
  }

  int numThreads() const { return num_threads; }
  int syncQueueSize() const { return sync_queue_size; }
  int syncToleranceNanosecond() const { return sync_tolerance_nanosecond; }
  bool syncPolicyApprox() const { return sync_policy_approx; }
  const std::string& outputTopic() const { return output.topic; }
  const std::string& outputFrameId() const { return output.frame_id; }
  const std::string& lidarTopic(const std::string& lidar_name) const {
    return lidars.at(lidar_name).topic;
  }
  const Eigen::Matrix4d& extrinsicMatrix(const std::string& lidar_name) const {
    return lidars.at(lidar_name).extrinsic_matrix;
  }

 private:
  int num_threads{4};
  int sync_queue_size{10};
  int sync_tolerance_nanosecond{1};
  bool sync_policy_approx{false};
  OutputConfig output{};
  std::map<std::string, LidarConfig> lidars{};
};

}  // namespace makinami
