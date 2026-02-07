#include <makinami/makinami_nodelet.hpp>

namespace makinami {

template <typename PointCloudT, typename CallbackT>
void MakinamiNodelet::setupSynchronizer(CallbackT callback) {
  using SubscriberType = SubscriberT<PointCloudT>;
  using ApproxPolicy = ApproxSyncPolicy<PointCloudT>;
  using ExactPolicy = ExactSyncPolicy<PointCloudT>;
  using ApproxSync = SynchronizerT<ApproxPolicy>;
  using ExactSync = SynchronizerT<ExactPolicy>;

  subscriber_master = boost::make_shared<SubscriberType>(
      pnh, config->lidarTopic("master_lidar"), 10);
  subscriber_slave = boost::make_shared<SubscriberType>(
      pnh, config->lidarTopic("slave_lidar"), 10);

  NODELET_INFO("Subscribing %s messages.",
               ros::message_traits::DataType<PointCloudT>::value());
  NODELET_INFO("master lidar topic: %s",
               config->lidarTopic("master_lidar").c_str());
  NODELET_INFO("slave lidar topic: %s",
               config->lidarTopic("slave_lidar").c_str());

  auto ppmaster =
      boost::get<boost::shared_ptr<SubscriberType>>(&subscriber_master);
  auto ppslave =
      boost::get<boost::shared_ptr<SubscriberType>>(&subscriber_slave);

  if (!ppmaster || !ppslave || !*ppmaster || !*ppslave) {
    NODELET_ERROR("Failed to get valid subscribers from variant");
    return;
  }

  try {
    if (config->syncPolicyApprox()) {
      auto approx_sync =
          boost::make_shared<ApproxSync>(ApproxPolicy(config->syncQueueSize()));
      approx_sync->connectInput(**ppmaster, **ppslave);
      approx_sync->setMaxIntervalDuration(
          ros::Duration(config->syncToleranceNanosecond() * 1e-9));
      approx_sync->registerCallback(callback);
      synchronizer = approx_sync;

      NODELET_INFO("Using ApproximateTime sync policy.");
      NODELET_INFO("Sync tolerance set to %d ns.",
                   config->syncToleranceNanosecond());
    } else {
      auto exact_sync =
          boost::make_shared<ExactSync>(ExactPolicy(config->syncQueueSize()));
      exact_sync->connectInput(**ppmaster, **ppslave);
      exact_sync->registerCallback(callback);
      synchronizer = exact_sync;

      NODELET_INFO("Using ExactTime sync policy.");
      NODELET_INFO("Sync tolerance set to %d ns.",
                   config->syncToleranceNanosecond());
    }
  } catch (...) {
  }
}

void MakinamiNodelet::onInit() {
  nh = getNodeHandle();
  pnh = getPrivateNodeHandle();

  {  // init parameters
    std::string config_file;
    try {
      pnh.getParam("config_file", config_file);
      config = std::make_shared<makinami::ConfigServer>();
      config->initializeParameters(config_file);
      extrinsic_matrix = config->extrinsicMatrix("slave_lidar");
      NODELET_INFO("Parameters initialized from %s", config_file.c_str());
    } catch (...) {
    }
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

  {  // init synchronizer
    if (config->livoxCustomMsg()) {
      setupSynchronizer<livox_ros_driver2::CustomMsg>(
          boost::bind(&MakinamiNodelet::syncLivoxCustomCallback, this, _1, _2));
    } else {
      setupSynchronizer<sensor_msgs::PointCloud2>(
          boost::bind(&MakinamiNodelet::syncPointCloud2Callback, this, _1, _2));
    }
  }

  mixed_publisher = boost::make_shared<ros::Publisher>(
      nh.advertise<sensor_msgs::PointCloud2>(config->outputTopic(), 10));
}

void MakinamiNodelet::syncLivoxCustomCallback(
    const livox_ros_driver2::CustomMsg::ConstPtr& msg_master,
    const livox_ros_driver2::CustomMsg::ConstPtr& msg_slave) {
  std::lock_guard<std::mutex> lock(cloud_mutex);
}

void MakinamiNodelet::syncPointCloud2Callback(
    const sensor_msgs::PointCloud2::ConstPtr& msg_master,
    const sensor_msgs::PointCloud2::ConstPtr& msg_slave) {
  std::lock_guard<std::mutex> lock(cloud_mutex);

  using LivoxPointCloudT = pcl::PointCloud<livox_ros::Point>;
  LivoxPointCloudT::Ptr cloud_master(new LivoxPointCloudT());
  LivoxPointCloudT::Ptr cloud_slave(new LivoxPointCloudT());

  pcl::fromROSMsg(*msg_master, *cloud_master);
  pcl::fromROSMsg(*msg_slave, *cloud_slave);

  cloud_master->reserve(cloud_master->size() + cloud_slave->size());

  pcl::transformPointCloud(*cloud_slave, *cloud_slave, extrinsic_matrix);
  *cloud_master += *cloud_slave;

  sensor_msgs::PointCloud2::Ptr msg_mixed(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*cloud_master, *msg_mixed);
  msg_mixed->header = msg_master->header;
  mixed_publisher->publish(msg_mixed);

  NODELET_DEBUG("Published mixed cloud with %zu points", cloud_master->size());
}
}  // namespace makinami

PLUGINLIB_EXPORT_CLASS(makinami::MakinamiNodelet, nodelet::Nodelet)