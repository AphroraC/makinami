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
#include <ros/message_traits.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <memory>
#include <mutex>
#include <string>

#include "livox/custom_msg.hpp"
#include "livox/pointcloud2.hpp"
#include "makinami/config_server.hpp"

namespace makinami {

template <typename PointCloudT>
using SubscriberT = message_filters::Subscriber<PointCloudT>;

using SubscriberVariantT =
    boost::variant<boost::shared_ptr<SubscriberT<livox_ros_driver2::CustomMsg>>,
                   boost::shared_ptr<SubscriberT<sensor_msgs::PointCloud2>>>;

template <typename PointCloudT>
using ApproxSyncPolicy =
    message_filters::sync_policies::ApproximateTime<PointCloudT, PointCloudT>;

template <typename PointCloudT>
using ExactSyncPolicy =
    message_filters::sync_policies::ExactTime<PointCloudT, PointCloudT>;

template <typename SyncPolicyT>
using SynchronizerT = message_filters::Synchronizer<SyncPolicyT>;

using SynchronizerVariantT = boost::variant<
    boost::shared_ptr<
        SynchronizerT<ApproxSyncPolicy<livox_ros_driver2::CustomMsg>>>,
    boost::shared_ptr<
        SynchronizerT<ApproxSyncPolicy<sensor_msgs::PointCloud2>>>,
    boost::shared_ptr<
        SynchronizerT<ExactSyncPolicy<livox_ros_driver2::CustomMsg>>>,
    boost::shared_ptr<
        SynchronizerT<ExactSyncPolicy<sensor_msgs::PointCloud2>>>>;

class MakinamiNodelet : public nodelet::Nodelet {
 public:
  MakinamiNodelet() = default;
  virtual ~MakinamiNodelet() override = default;
  void onInit();

 protected:
  template <typename PointCloudT, typename CallbackT>
  void setupSynchronizer(CallbackT callback);
  void syncLivoxCustomCallback(
      const livox_ros_driver2::CustomMsg::ConstPtr& msg_master,
      const livox_ros_driver2::CustomMsg::ConstPtr& msg_slave);
  void syncPointCloud2Callback(
      const sensor_msgs::PointCloud2::ConstPtr& msg_master,
      const sensor_msgs::PointCloud2::ConstPtr& msg_slave);

 private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  boost::shared_ptr<ros::Publisher> mixed_publisher{nullptr};
  boost::shared_ptr<ros::AsyncSpinner> async_spinner{nullptr};
  boost::shared_ptr<ros::CallbackQueue> private_callback_queue{nullptr};

  SubscriberVariantT subscriber_master{};
  SubscriberVariantT subscriber_slave{};
  SynchronizerVariantT synchronizer{};

  Eigen::Matrix4d extrinsic_matrix{Eigen::Matrix4d::Identity()};

  std::mutex cloud_mutex;
  std::shared_ptr<makinami::ConfigServer> config{nullptr};
};
}  // namespace makinami
