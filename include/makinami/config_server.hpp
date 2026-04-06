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
  void initializeParameters(const std::string &config_file) {
    try {
      std::ifstream file(config_file);
      json data = json::parse(file, nullptr, true, true);

      num_threads = data["num_threads"];
      sync_queue_size = data["sync_queue_size"];
      sync_tolerance_nanosecond = data["sync_tolerance_nanosecond"];
      sync_policy_approx = data["sync_policy_approx"];
      livox_custom_msg = data["livox_custom_msg"];
      output.topic = data["output_params"]["topic"];
      output.frame_id = data["output_params"]["frame_id"];

      for (const auto &lidar : data["lidar_params"]) {
        std::string name = lidar["name"];
        LidarConfig param;
        param.topic = lidar["topic"];
        Eigen::Matrix4d matrix;
        matrix << lidar["matrix"][0], lidar["matrix"][1], lidar["matrix"][2],
            lidar["matrix"][3], lidar["matrix"][4], lidar["matrix"][5],
            lidar["matrix"][6], lidar["matrix"][7], lidar["matrix"][8],
            lidar["matrix"][9], lidar["matrix"][10], lidar["matrix"][11],
            lidar["matrix"][12], lidar["matrix"][13], lidar["matrix"][14],
            lidar["matrix"][15];

        std::cout << "Lidar " << name << " extrinsic matrix:\n"
                  << matrix << std::endl;
        param.extrinsic_matrix = matrix;

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
  bool livoxCustomMsg() const { return livox_custom_msg; }
  std::string outputTopic() const { return output.topic; }
  std::string outputFrameId() const { return output.frame_id; }
  std::string lidarTopic(const std::string &lidar_name) const {
    return lidars.at(lidar_name).topic;
  }
  Eigen::Matrix4d extrinsicMatrix(const std::string &lidar_name) const {
    return lidars.at(lidar_name).extrinsic_matrix;
  }

private:
  int num_threads{2};
  int sync_queue_size{10};
  int sync_tolerance_nanosecond{1};
  bool sync_policy_approx{false};
  bool livox_custom_msg{false};
  OutputConfig output{};
  std::map<std::string, LidarConfig> lidars{};
};

} // namespace makinami
