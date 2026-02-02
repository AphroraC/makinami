#pragma once
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <memory>
#include <mutex>
#include <string>

#include "livox/custom_msg.hpp"
#include "makinami/config_server.hpp"

namespace makinami {
using PointCloudT = livox_ros_driver2::CustomMsg;
using SubscriberT = message_filters::Subscriber<PointCloudT>;
using ApproxSyncPolicy =
    message_filters::sync_policies::ApproximateTime<PointCloudT, PointCloudT>;
using ExactSyncPolicy =
    message_filters::sync_policies::ExactTime<PointCloudT, PointCloudT>;
using ApproxSynchronizer = message_filters::Synchronizer<ApproxSyncPolicy>;
using ExactSynchronizer = message_filters::Synchronizer<ExactSyncPolicy>;

class MakinamiNodelet : public nodelet::Nodelet {
 public:
  MakinamiNodelet() = default;
  virtual ~MakinamiNodelet() override = default;
  void onInit();

 protected:
  void syncCloudCallback(const makinami::PointCloudT::ConstPtr& msg_master,
                         const makinami::PointCloudT::ConstPtr& msg_slave);

 private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Publisher mixed_cloud_pub;

  boost::shared_ptr<ros::AsyncSpinner> async_spinner{nullptr};
  boost::shared_ptr<ros::CallbackQueue> private_callback_queue{nullptr};
  boost::shared_ptr<makinami::ApproxSynchronizer> approx_synchronizer{nullptr};
  boost::shared_ptr<makinami::ExactSynchronizer> exact_synchronizer{nullptr};
  boost::shared_ptr<makinami::SubscriberT> master_lidar_sub{nullptr};
  boost::shared_ptr<makinami::SubscriberT> slave_lidar_sub{nullptr};

  Eigen::Matrix4d extrinsic_matrix{Eigen::Matrix4d::Identity()};

  std::mutex cloud_mutex;
  std::shared_ptr<makinami::ConfigServer> config{nullptr};
};
}  // namespace makinami
