#include <makinami/makinami_nodelet.hpp>

namespace makinami {

void makinami::MakinamiNodelet::onInit() {
  nh = getNodeHandle();
  pnh = getPrivateNodeHandle();

  {
    std::string config_file;
    pnh.getParam("config_file", config_file);
    config = std::make_shared<makinami::ConfigServer>();
    config->initializeParameters(config_file);
    extrinsic_matrix = config->extrinsicMatrix("slave_lidar");
    NODELET_INFO("Parameters initialized from %s", config_file.c_str());
  }

  {  // init callback queue
    private_callback_queue = boost::make_shared<ros::CallbackQueue>();
    pnh.setCallbackQueue(private_callback_queue.get());
    async_spinner = boost::make_shared<ros::AsyncSpinner>(
        config->numThreads(), private_callback_queue.get());
    async_spinner->start();
    NODELET_INFO("Async spinner started with %d threads.",
                 config->numThreads());
  }

  {  // init message filters subscribers
    master_lidar_sub = boost::make_shared<makinami::SubscriberT>(
        pnh, config->lidarTopic("master_lidar"), 10);
    slave_lidar_sub = boost::make_shared<makinami::SubscriberT>(
        pnh, config->lidarTopic("slave_lidar"), 10);
    NODELET_INFO("Subscribed to master lidar topic: %s",
                 config->lidarTopic("master_lidar").c_str());
    NODELET_INFO("Subscribed to slave lidar topic: %s",
                 config->lidarTopic("slave_lidar").c_str());
  }

  {  // init time synchronizer
    if (config->syncPolicyApprox()) {
      approx_synchronizer = boost::make_shared<makinami::ApproxSynchronizer>(
          ApproxSyncPolicy(config->syncQueueSize()));
      approx_synchronizer->connectInput(*master_lidar_sub, *slave_lidar_sub);
      approx_synchronizer->setMaxIntervalDuration(
          ros::Duration(config->syncToleranceNanosecond() * 1e-9));
      approx_synchronizer->registerCallback(
          boost::bind(&MakinamiNodelet::syncCloudCallback, this, _1, _2));
      NODELET_INFO("Using ApproximateTime sync policy.");
      NODELET_INFO("Sync tolerance set to %d ns.",
                   config->syncToleranceNanosecond());
    } else {
      exact_synchronizer = boost::make_shared<makinami::ExactSynchronizer>(
          ExactSyncPolicy(config->syncQueueSize()));
      exact_synchronizer->connectInput(*master_lidar_sub, *slave_lidar_sub);
      exact_synchronizer->registerCallback(
          boost::bind(&MakinamiNodelet::syncCloudCallback, this, _1, _2));
      NODELET_INFO("Using ExactTime sync policy.");
      NODELET_INFO("Sync tolerance set to %d ns.",
                   config->syncToleranceNanosecond());
    }
  }

  mixed_cloud_pub =
      nh.advertise<makinami::PointCloudT>(config->outputTopic(), 10);
}

void makinami::MakinamiNodelet::syncCloudCallback(
    const makinami::PointCloudT::ConstPtr& msg_master,
    const makinami::PointCloudT::ConstPtr& msg_slave) {
  std::lock_guard<std::mutex> lock(cloud_mutex);
}
}  // namespace makinami

PLUGINLIB_EXPORT_CLASS(makinami::MakinamiNodelet, nodelet::Nodelet)