/* includes //{ */

#include <ros/init.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <octomap/OcTreeNode.h>
#include <octomap/octomap_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>

#include <eigen3/Eigen/Eigen>

#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>

#include <Eigen/Geometry>

#include <octomap_msgs/BoundingBoxQueryRequest.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include <filesystem>

#include <mrs_octomap_server/conversions.h>

#include <laser_geometry/laser_geometry.h>

#include <cmath>

//}

namespace ph = std::placeholders;

namespace mrs_octomap_server
{

/* defines //{ */

using vec3s_t = Eigen::Matrix<float, 3, -1>;
using vec3_t  = Eigen::Vector3f;

struct xyz_lut_t
{
  vec3s_t directions;  // a matrix of normalized direction column vectors
  vec3s_t offsets;     // a matrix of offset vectors
};

//}

/* class OctomapServer //{ */

class OctomapServer : public nodelet::Nodelet {

public:
#ifdef COLOR_OCTOMAP_SERVER
  using PCLPoint      = pcl::PointXYZRGB;
  using PCLPointCloud = pcl::PointCloud<PCLPoint>;
  using OcTreeT       = octomap::ColorOcTree;
#else
  using PCLPoint      = pcl::PointXYZ;
  using PCLPointCloud = pcl::PointCloud<PCLPoint>;
  using OcTreeT       = octomap::OcTree;
#endif

  virtual void onInit();

  bool callbackLoadMap(mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& resp);
  bool callbackSaveMap(mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& resp);

  bool callbackResetMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  virtual void insertCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual void insertLaserScanCallback(const sensor_msgs::LaserScanConstPtr& scan);
  virtual bool loadFromFile(const std::string& filename);
  virtual bool saveToFile(const std::string& filename);

protected:
  ros::NodeHandle nh_;
  bool            is_initialized_;

  // | -------------------- topic subscribers ------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> m_pointCloudSub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>>   m_laserScanSub;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher pub_map_global_full_;
  ros::Publisher pub_map_global_binary_;

  ros::Publisher pub_map_local_full_;
  ros::Publisher pub_map_local_binary_;

  // | -------------------- service serviers -------------------- |

  ros::ServiceServer ss_reset_map_;
  ros::ServiceServer ss_save_map_;
  ros::ServiceServer ss_load_map_;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_global_map_;
  double     _global_map_rate_;
  void       timerGlobalMap([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_local_map_;
  void       timerLocalMap([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_persistency_;
  void       timerPersistency([[maybe_unused]] const ros::TimerEvent& event);

  // | ----------------------- parameters ----------------------- |

  bool _simulation_;

  bool        _persistency_enabled_;
  std::string _persistency_map_name_;
  double      _persistency_save_time_;

  bool _global_map_publish_full_;
  bool _global_map_publish_binary_;

  double _local_map_size_;
  bool   _local_map_enabled_;
  bool   _local_map_publish_full_;
  bool   _local_map_publish_binary_;

  tf2_ros::Buffer                             m_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

  std::shared_ptr<OcTreeT> octree_;
  std::mutex               mutex_octree_;

  std::shared_ptr<OcTreeT> octree_local_;
  std::mutex               mutex_octree_local_;

  double     avg_time_cloud_insertion_ = 0;
  std::mutex mutex_avg_time_cloud_insertion_;

  double     avg_time_local_map_processing_ = 0;
  std::mutex mutex_avg_time_local_map_processing_;

  octomap::KeyRay    m_keyRay;  // temp storage for ray casting
  octomap::OcTreeKey m_updateBBXMin;
  octomap::OcTreeKey m_updateBBXMax;

  double      m_maxRange;
  std::string _world_frame_;
  std::string _robot_frame_;
  double      m_res;
  unsigned    m_treeDepth;
  unsigned    m_maxTreeDepth;
  double      m_minSizeX;
  double      m_minSizeY;
  bool        m_filterSpeckles;
  bool        _global_map_compress_;
  std::string _map_path_;

  bool   m_filterGroundPlane;
  double m_ZGroundFilterDistance;

  double _local_map_distance_;
  double _local_map_rate_;

  bool   _unknown_rays_update_free_space_;
  double _unknown_rays_distance_;

  laser_geometry::LaserProjection projector_;

  bool copyInsideBBX(std::shared_ptr<OcTreeT>& from, std::shared_ptr<OcTreeT>& to, const octomap::point3d& p_min, const octomap::point3d& p_max);

  bool createLocalMap(const std::string frame_id, const double radius, std::shared_ptr<OcTreeT>& octree);

  inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
    for (unsigned i = 0; i < 3; ++i)
      min[i] = std::min(in[i], min[i]);
  };

  inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
    for (unsigned i = 0; i < 3; ++i)
      max[i] = std::max(in[i], max[i]);
  };

  virtual void insertPointCloud(const geometry_msgs::Vector3& sensorOrigin, const PCLPointCloud::ConstPtr& cloud, const PCLPointCloud::ConstPtr& free_cloud);

  void initializeLidarLUT(const size_t w, const size_t h);

  xyz_lut_t m_sensor_3d_xyz_lut;
  bool      m_sensor_3d_params_enabled;
  float     m_sensor_3d_vfov;
  int       m_sensor_3d_vrays;
  int       m_sensor_3d_hrays;
};

//}

/* onInit() //{ */

void OctomapServer::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  /* params //{ */

  mrs_lib::ParamLoader param_loader(nh_, ros::this_node::getName());

  param_loader.loadParam("simulation", _simulation_);

  param_loader.loadParam("persistency/enabled", _persistency_enabled_);
  param_loader.loadParam("persistency/save_time", _persistency_save_time_);
  param_loader.loadParam("persistency/map_name", _persistency_map_name_);

  param_loader.loadParam("global_map/rate", _global_map_rate_);
  param_loader.loadParam("global_map/compress", _global_map_compress_);
  param_loader.loadParam("global_map/publish_full", _global_map_publish_full_);
  param_loader.loadParam("global_map/publish_binary", _global_map_publish_binary_);

  param_loader.loadParam("local_map/enabled", _local_map_enabled_);
  param_loader.loadParam("local_map/distance", _local_map_distance_);
  param_loader.loadParam("local_map/rate", _local_map_rate_);
  param_loader.loadParam("local_map/publish_full", _local_map_publish_full_);
  param_loader.loadParam("local_map/publish_binary", _local_map_publish_binary_);

  param_loader.loadParam("resolution", m_res);
  param_loader.loadParam("world_frame_id", _world_frame_);
  param_loader.loadParam("robot_frame_id", _robot_frame_);

  param_loader.loadParam("map_path", _map_path_);

  param_loader.loadParam("unknown_rays/update_free_space", _unknown_rays_update_free_space_);
  param_loader.loadParam("unknown_rays/ray_distance", _unknown_rays_distance_);

  param_loader.loadParam("sensor_params_3d/enabled", m_sensor_3d_params_enabled);
  param_loader.loadParam("sensor_params_3d/vertical_fov_angle", m_sensor_3d_vfov);
  param_loader.loadParam("sensor_params_3d/vertical_rays", m_sensor_3d_vrays);
  param_loader.loadParam("sensor_params_3d/horizontal_rays", m_sensor_3d_hrays);

  double probHit, probMiss, thresMin, thresMax;
  param_loader.loadParam("sensor_model/hit", probHit);
  param_loader.loadParam("sensor_model/miss", probMiss);
  param_loader.loadParam("sensor_model/min", thresMin);
  param_loader.loadParam("sensor_model/max", thresMax);
  param_loader.loadParam("sensor_model/max_range", m_maxRange);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", ros::this_node::getName().c_str());
    ros::requestShutdown();
  }

  //}

  /* check params //{ */


  if (_local_map_enabled_ && _local_map_distance_ < m_maxRange) {
    std::string msg = std::string("You enabled using only the local map. ") +
                      "However, the local distance for the map is lower than the maximal sensor range. " +
                      "Defaulting the local distance for the map to the maximal sensor range.";
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.c_str());
    _local_map_distance_ = m_maxRange;
  }

  //}

  /* initialize sensor LUT model //{ */

  if (m_sensor_3d_params_enabled) {

    initializeLidarLUT(m_sensor_3d_hrays, m_sensor_3d_vrays);
  }

  //}

  /* initialize octomap object & params //{ */

  octree_ = std::make_shared<OcTreeT>(m_res);
  octree_->setProbHit(probHit);
  octree_->setProbMiss(probMiss);
  octree_->setClampingThresMin(thresMin);
  octree_->setClampingThresMax(thresMax);

  octree_local_ = std::make_shared<OcTreeT>(m_res);
  octree_local_->setProbHit(probHit);
  octree_local_->setProbMiss(probMiss);
  octree_local_->setClampingThresMin(thresMin);
  octree_local_->setClampingThresMax(thresMax);

  m_treeDepth    = octree_->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;

  //}

  /* persistency //{ */

  if (_persistency_enabled_) {

    bool success = loadFromFile(_persistency_map_name_);

    if (success) {
      ROS_INFO("[OctomapServer]: loaded persistency map");
    } else {

      ROS_ERROR("[OctomapServer]: failed to load the persistency map, turning persistency off");

      _persistency_enabled_ = false;
    }
  }

  //}

  /* tf_listener //{ */

  this->m_buffer.setUsingDedicatedThread(true);
  this->m_tfListener = std::make_unique<tf2_ros::TransformListener>(m_buffer, ros::this_node::getName());

  //}

  /* publishers //{ */

  pub_map_global_full_   = nh_.advertise<octomap_msgs::Octomap>("octomap_global_full_out", 1);
  pub_map_global_binary_ = nh_.advertise<octomap_msgs::Octomap>("octomap_global_binary_out", 1);

  pub_map_local_full_   = nh_.advertise<octomap_msgs::Octomap>("octomap_local_full_out", 1);
  pub_map_local_binary_ = nh_.advertise<octomap_msgs::Octomap>("octomap_local_binary_out", 1);

  //}

  /* subscribers //{ */

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "OctomapServer";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");

  // Point Cloud
  m_pointCloudSub = std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, "point_cloud_in", 5);
  m_pointCloudSub->registerCallback(std::bind(&OctomapServer::insertCloudCallback, this, ph::_1));

  // Laser scan
  m_laserScanSub = std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, "laser_scan_in", 5);
  m_laserScanSub->registerCallback(std::bind(&OctomapServer::insertLaserScanCallback, this, ph::_1));

  //}

  /* service servers //{ */

  this->ss_reset_map_ = nh_.advertiseService("reset_map_in", &OctomapServer::callbackResetMap, this);
  this->ss_save_map_  = nh_.advertiseService("save_map_in", &OctomapServer::callbackSaveMap, this);
  this->ss_load_map_  = nh_.advertiseService("load_map_in", &OctomapServer::callbackLoadMap, this);

  //}

  /* timers //{ */

  timer_global_map_ = nh_.createTimer(ros::Rate(_global_map_rate_), &OctomapServer::timerGlobalMap, this);

  if (_local_map_enabled_) {
    timer_local_map_ = nh_.createTimer(ros::Rate(_local_map_rate_), &OctomapServer::timerLocalMap, this);
  }

  if (_persistency_enabled_) {
    timer_persistency_ = nh_.createTimer(ros::Rate(1.0 / _persistency_save_time_), &OctomapServer::timerPersistency, this);
  }

  //}

  is_initialized_ = true;

  ROS_INFO("[%s]: Initialized", ros::this_node::getName().c_str());
}

//}

// | --------------------- topic callbacks -------------------- |

/* insertLaserScanCallback() //{ */

void OctomapServer::insertLaserScanCallback(const sensor_msgs::LaserScanConstPtr& scan) {

  if (!is_initialized_) {
    return;
  }

  PCLPointCloud::Ptr pc              = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = boost::make_shared<PCLPointCloud>();

  Eigen::Matrix4f                 sensorToWorld;
  geometry_msgs::TransformStamped sensorToWorldTf;

  try {

    if (!this->m_buffer.canTransform(_world_frame_, scan->header.frame_id, scan->header.stamp)) {
      sensorToWorldTf = this->m_buffer.lookupTransform(_world_frame_, scan->header.frame_id, ros::Time(0));
    } else {
      sensorToWorldTf = this->m_buffer.lookupTransform(_world_frame_, scan->header.frame_id, scan->header.stamp);
    }

    pcl_ros::transformAsMatrix(sensorToWorldTf.transform, sensorToWorld);
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), ex.what());
    return;
  }

  // laser scan to point cloud
  sensor_msgs::PointCloud2 ros_cloud;
  projector_.projectLaser(*scan, ros_cloud);
  pcl::fromROSMsg(ros_cloud, *pc);

  // compute free rays, if required
  if (_unknown_rays_update_free_space_) {

    sensor_msgs::LaserScan free_scan = *scan;

    double free_scan_distance = (scan->range_max - 1.0) < _unknown_rays_distance_ ? (scan->range_max - 1.0) : _unknown_rays_distance_;

    for (int i = 0; i < scan->ranges.size(); i++) {
      if (scan->ranges[i] > scan->range_max || scan->ranges[i] < scan->range_min) {
        free_scan.ranges[i] = scan->range_max - 1.0;  // valid under max range
      } else {
        free_scan.ranges[i] = scan->range_min - 1.0;  // definitely invalid
      }
    }

    sensor_msgs::PointCloud2 free_cloud;
    projector_.projectLaser(free_scan, free_cloud);

    pcl::fromROSMsg(free_cloud, *free_vectors_pc);
  }

  free_vectors_pc->header = pc->header;

  // transform to the map frame

  pcl::transformPointCloud(*pc, *pc, sensorToWorld);
  pcl::transformPointCloud(*free_vectors_pc, *free_vectors_pc, sensorToWorld);

  pc->header.frame_id              = _world_frame_;
  free_vectors_pc->header.frame_id = _world_frame_;

  insertPointCloud(sensorToWorldTf.transform.translation, pc, free_vectors_pc);

  const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation);
}

//}

/* insertCloudCallback() //{ */

void OctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud) {

  if (!is_initialized_) {
    return;
  }

  ros::Time time_start = ros::Time::now();

  PCLPointCloud::Ptr pc              = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = boost::make_shared<PCLPointCloud>();
  pcl::fromROSMsg(*cloud, *pc);

  Eigen::Matrix4f                 sensorToWorld;
  geometry_msgs::TransformStamped sensorToWorldTf;

  try {

    ros::Time time = cloud->header.stamp;

    if (!this->m_buffer.canTransform(_world_frame_, cloud->header.frame_id, time)) {
      time = ros::Time(0);
    }

    sensorToWorldTf = this->m_buffer.lookupTransform(_world_frame_, cloud->header.frame_id, time);
    pcl_ros::transformAsMatrix(sensorToWorldTf.transform, sensorToWorld);
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), ex.what());
    return;
  }

  // compute free rays, if required
  if (_unknown_rays_update_free_space_) {

    /* Eigen::Affine3d s2w = tf2::transformToEigen(sensorToWorldTf); */

    /* const auto tf_rot = s2w.rotation(); */
    /* // origin of all rays of the lidar sensor */
    /* const vec3_t origin_pt = s2w.translation(); */

    // go through all points in the cloud and update voxels in the helper voxelmap that the rays
    // from the sensor origin to the point go through according to how long part of the ray
    // intersects the voxel
    for (int i = 0; i < pc->size(); i++) {

      pcl::PointXYZ pt = pc->at(i);

      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {

        const vec3_t ray_vec = m_sensor_3d_xyz_lut.directions.col(i);

        if (ray_vec(2) > 0.0) {
          pt.x = ray_vec(0) * float(_unknown_rays_distance_);
          pt.y = ray_vec(1) * float(_unknown_rays_distance_);
          pt.z = ray_vec(2) * float(_unknown_rays_distance_);

          free_vectors_pc->push_back(pt);
        }
      }
    }
  }

  free_vectors_pc->header = pc->header;

  // transform to the map frame

  pcl::transformPointCloud(*pc, *pc, sensorToWorld);
  pcl::transformPointCloud(*free_vectors_pc, *free_vectors_pc, sensorToWorld);

  pc->header.frame_id              = _world_frame_;
  free_vectors_pc->header.frame_id = _world_frame_;

  insertPointCloud(sensorToWorldTf.transform.translation, pc, free_vectors_pc);

  const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation);

  {
    std::scoped_lock lock(mutex_avg_time_cloud_insertion_);

    ros::Time time_end = ros::Time::now();

    double exec_duration = (time_end - time_start).toSec();

    double coef               = 0.95;
    avg_time_cloud_insertion_ = coef * avg_time_cloud_insertion_ + (1.0 - coef) * exec_duration;

    ROS_INFO_THROTTLE(5.0, "[OctomapServer]: avg cloud insertion time = %.3f sec", avg_time_cloud_insertion_);
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackLoadMap() //{ */

bool OctomapServer::callbackLoadMap([[maybe_unused]] mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  ROS_INFO("[OctomapServer]: loading map");

  bool success = loadFromFile(req.value);

  if (success) {

    res.success = true;
    res.message = "map loaded";

  } else {

    res.success = false;
    res.message = "map loading error";
  }

  return true;
}

//}

/* callbackSaveMap() //{ */

bool OctomapServer::callbackSaveMap([[maybe_unused]] mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  bool success = saveToFile(req.value);

  if (success) {

    res.message = "map saved";
    res.success = true;

  } else {

    res.message = "map saving failed";
    res.success = false;
  }

  return true;
}

//}

/* callbackResetMap() //{ */

bool OctomapServer::callbackResetMap([[maybe_unused]] std_srvs::Empty::Request& req, [[maybe_unused]] std_srvs::Empty::Response& resp) {

  {
    std::scoped_lock lock(mutex_octree_);

    octree_->clear();
  }

  ROS_INFO("[OctomapServer]: octomap cleared");

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerGlobalMap() //{ */

void OctomapServer::timerGlobalMap([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: full map timer spinning");

  std::scoped_lock lock(mutex_octree_);

  size_t octomap_size = octree_->size();

  if (octomap_size <= 1) {
    ROS_WARN("[%s]: Nothing to publish, octree is empty", ros::this_node::getName().c_str());
    return;
  }

  if (_global_map_compress_) {
    octree_->prune();
  }

  if (pub_map_global_full_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    if (octomap_msgs::fullMapToMsg(*octree_, map)) {
      pub_map_global_full_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing global octomap to full representation");
    }
  }

  if (_global_map_publish_binary_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    if (octomap_msgs::binaryMapToMsg(*octree_, map)) {
      pub_map_global_binary_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing global octomap to binary representation");
    }
  }
}

//}

/* timerLocalMap() //{ */

void OctomapServer::timerLocalMap([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: local map timer spinning");

  std::scoped_lock lock(mutex_octree_local_);

  bool success = createLocalMap(_robot_frame_, _local_map_distance_, octree_local_);

  if (!success) {
    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: failed to create the local map");
    return;
  }

  size_t octomap_size = octree_local_->size();

  if (octomap_size <= 1) {
    ROS_WARN("[%s]: Nothing to publish, octree is empty", ros::this_node::getName().c_str());
    return;
  }

  if (pub_map_global_full_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    if (octomap_msgs::fullMapToMsg(*octree_local_, map)) {
      pub_map_local_full_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing local octomap to full representation");
    }
  }

  if (_global_map_publish_binary_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    if (octomap_msgs::binaryMapToMsg(*octree_local_, map)) {
      pub_map_local_binary_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing local octomap to binary representation");
    }
  }
}

//}

/* timerPersistency() //{ */

void OctomapServer::timerPersistency([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: persistency timer spinning");

  if (!sh_control_manager_diag_.hasMsg()) {

    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: missing control manager diagnostics, won't save the map automatically!");
    return;

  } else {

    ros::Time last_time = sh_control_manager_diag_.lastMsgTime();

    if ((ros::Time::now() - last_time).toSec() > 1.0) {
      ROS_WARN_THROTTLE(1.0, "[OctomapServer]: control manager diagnostics too old, won't save the map automatically!");
      return;
    }
  }

  mrs_msgs::ControlManagerDiagnosticsConstPtr control_manager_diag = sh_control_manager_diag_.getMsg();

  if (control_manager_diag->flying_normally) {

    ROS_INFO_THROTTLE(1.0, "[OctomapServer]: saving the map");

    bool success = saveToFile(_persistency_map_name_);

    if (success) {
      ROS_INFO("[OctomapServer]: persistent map saved");
    } else {
      ROS_ERROR("[OctomapServer]: failed to saved persistent map");
    }
  }
}

//}

// | ------------------------ routines ------------------------ |

/* insertPointCloud() //{ */

void OctomapServer::insertPointCloud(const geometry_msgs::Vector3& sensorOriginTf, const PCLPointCloud::ConstPtr& cloud,
                                     const PCLPointCloud::ConstPtr& free_vectors_cloud) {

  std::scoped_lock lock(mutex_octree_);

  const octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

  if (!octree_->coordToKeyChecked(sensorOrigin, m_updateBBXMin) || !octree_->coordToKeyChecked(sensorOrigin, m_updateBBXMax)) {
    ROS_ERROR_STREAM("Could not generate Key for origin " << sensorOrigin);
  }

  const float free_space_ray_len = 15.0;

  // instead of direct scan insertion, compute probabilistic update
  octomap::KeySet free_cells, occupied_cells;
  const bool      free_space_bounded = free_space_ray_len > 0.0f;

  // all points: free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = cloud->begin(); it != cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    octomap::KeyRay  keyRay;
    const float      point_distance = float((measured_point - sensorOrigin).norm());

    octomap::OcTreeKey key;
    if (octree_->coordToKeyChecked(measured_point, key)) {

      occupied_cells.insert(key);

      /* updateMinKey(key, m_updateBBXMin); */
      /* updateMaxKey(key, m_updateBBXMax); */
    }

    // move end point to distance min(free space ray len, current distance)
    measured_point = sensorOrigin + (measured_point - sensorOrigin).normalize() * std::min(free_space_ray_len, point_distance);

    // free cells
    if (octree_->computeRayKeys(sensorOrigin, measured_point, keyRay)) {
      free_cells.insert(keyRay.begin(), keyRay.end());
    }
  }

  for (PCLPointCloud::const_iterator it = free_vectors_cloud->begin(); it != free_vectors_cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    octomap::KeyRay  keyRay;

    // check if the ray intersects a cell in the occupied list
    if (octree_->computeRayKeys(sensorOrigin, measured_point, keyRay)) {

      bool                      ray_is_cool         = true;
      octomap::KeyRay::iterator alterantive_ray_end = keyRay.end();

      for (octomap::KeyRay::iterator it2 = keyRay.begin(), end = keyRay.end(); it2 != end; ++it2) {

        // check if the cell was spotted as occupied by a valid ray
        if (occupied_cells.find(*it2) != occupied_cells.end()) {
          ray_is_cool = false;
          break;
        }

        // check if the cell is occupied in a map
        auto node = octree_->search(*it2);
        if (node && octree_->isNodeOccupied(node)) {

          if (it2 == keyRay.begin()) {
            alterantive_ray_end = keyRay.begin();  // special case
          } else {
            alterantive_ray_end = it2 - 1;
          }

          break;
        }
      }

      if (ray_is_cool) {
        free_cells.insert(keyRay.begin(), alterantive_ray_end);
      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
  for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
    if (occupied_cells.find(*it) == occupied_cells.end()) {
      octree_->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {
    octree_->updateNode(*it, true);
  }
}

//}

/* initializeLidarLUT() //{ */

void OctomapServer::initializeLidarLUT(const size_t w, const size_t h) {

  const int                                       rangeCount         = w;
  const int                                       verticalRangeCount = h;
  std::vector<std::tuple<double, double, double>> coord_coeffs;
  const double                                    minAngle = 0.0;
  const double                                    maxAngle = 2.0 * M_PI;

  const double verticalMinAngle = -m_sensor_3d_vfov / 2.0;
  const double verticalMaxAngle = m_sensor_3d_vfov / 2.0;

  const double yDiff = maxAngle - minAngle;
  const double pDiff = verticalMaxAngle - verticalMinAngle;

  double yAngle_step = yDiff / (rangeCount - 1);

  double pAngle_step;
  if (verticalRangeCount > 1)
    pAngle_step = pDiff / (verticalRangeCount - 1);
  else
    pAngle_step = 0;

  coord_coeffs.reserve(rangeCount * verticalRangeCount);

  for (int i = 0; i < rangeCount; i++) {
    for (int j = 0; j < verticalRangeCount; j++) {

      // Get angles of ray to get xyz for point
      const double yAngle = i * yAngle_step + minAngle;
      const double pAngle = j * pAngle_step + verticalMinAngle;

      const double x_coeff = cos(pAngle) * cos(yAngle);
      const double y_coeff = cos(pAngle) * sin(yAngle);
      const double z_coeff = sin(pAngle);
      coord_coeffs.push_back({x_coeff, y_coeff, z_coeff});
    }
  }

  int it = 0;
  m_sensor_3d_xyz_lut.directions.resize(3, rangeCount * verticalRangeCount);
  m_sensor_3d_xyz_lut.offsets.resize(3, rangeCount * verticalRangeCount);

  for (int row = 0; row < verticalRangeCount; row++) {
    for (int col = 0; col < rangeCount; col++) {
      const auto [x_coeff, y_coeff, z_coeff] = coord_coeffs.at(col * verticalRangeCount + row);
      m_sensor_3d_xyz_lut.directions.col(it) = vec3_t(x_coeff, y_coeff, z_coeff);
      m_sensor_3d_xyz_lut.offsets.col(it)    = vec3_t(0, 0, 0);
      it++;
    }
  }
}

//}

/* loadFromFile() //{ */

bool OctomapServer::loadFromFile(const std::string& filename) {

  std::string file_path = _map_path_ + "/" + filename + ".ot";

  {
    std::scoped_lock lock(mutex_octree_);

    if (file_path.length() <= 3)
      return false;

    std::string suffix = file_path.substr(file_path.length() - 3, 3);

    if (suffix == ".bt") {
      if (!octree_->readBinary(file_path)) {
        return false;
      }
    } else if (suffix == ".ot") {

      auto tree = octomap::AbstractOcTree::read(file_path);
      if (!tree) {
        return false;
      }

      OcTreeT* octree = dynamic_cast<OcTreeT*>(tree);
      octree_         = std::shared_ptr<OcTreeT>(octree);

      if (!octree_) {
        ROS_ERROR("[OctomapServer]: could not read OcTree file");
        return false;
      }

    } else {
      return false;
    }

    m_treeDepth    = octree_->getTreeDepth();
    m_maxTreeDepth = m_treeDepth;
    m_res          = octree_->getResolution();

    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    octree_->getMetricMin(minX, minY, minZ);
    octree_->getMetricMax(maxX, maxY, maxZ);

    m_updateBBXMin[0] = octree_->coordToKey(minX);
    m_updateBBXMin[1] = octree_->coordToKey(minY);
    m_updateBBXMin[2] = octree_->coordToKey(minZ);

    m_updateBBXMax[0] = octree_->coordToKey(maxX);
    m_updateBBXMax[1] = octree_->coordToKey(maxY);
    m_updateBBXMax[2] = octree_->coordToKey(maxZ);
  }

  return true;
}

//}

/* saveToFile() //{ */

bool OctomapServer::saveToFile(const std::string& filename) {

  std::scoped_lock lock(mutex_octree_);

  std::string file_path        = _map_path_ + "/" + filename + ".ot";
  std::string tmp_file_path    = _map_path_ + "/tmp_" + filename + ".ot";
  std::string backup_file_path = _map_path_ + "/" + filename + "_backup.ot";

  try {
    std::filesystem::rename(file_path, backup_file_path);
  }
  catch (std::filesystem::filesystem_error& e) {
    ROS_ERROR("[OctomapServer]: failed to copy map to the backup path");
  }

  std::string suffix = file_path.substr(file_path.length() - 3, 3);

  if (!octree_->write(tmp_file_path)) {
    ROS_ERROR("[OctomapServer]: error writing to file '%s'", file_path.c_str());
    return false;
  }

  try {
    std::filesystem::rename(tmp_file_path, file_path);
  }
  catch (std::filesystem::filesystem_error& e) {
    ROS_ERROR("[OctomapServer]: failed to copy map to the backup path");
  }

  return true;
}

//}

/* copyInsideBBX() //{ */

bool OctomapServer::copyInsideBBX(std::shared_ptr<OcTreeT>& from, std::shared_ptr<OcTreeT>& to, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey minKey, maxKey;

  if (!from->coordToKeyChecked(p_min, minKey) || !from->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  /* from->expand(); */

  for (OcTreeT::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max), end = from->end_leafs_bbx(); it != end; ++it) {

    // check if outside of bbx:
    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = from->search(k);

    from->expandNode(node);
  }

  for (OcTreeT::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max), end = from->end_leafs_bbx(); it != end; ++it) {

    // check if outside of bbx:
    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = from->search(k);

    to->setNodeValue(k, node->getValue());
  }

  /* from->prune(); */
  /* to->prune(); */

  return true;
}

//}

/* createLocalMap() //{ */

bool OctomapServer::createLocalMap(const std::string frame_id, const double radius, std::shared_ptr<OcTreeT>& octree) {

  ros::Time time_start = ros::Time::now();

  // get the position of the robot frame in the map
  geometry_msgs::TransformStamped world_to_robot;

  try {

    if (!this->m_buffer.canTransform(_world_frame_, frame_id, ros::Time::now())) {
      world_to_robot = this->m_buffer.lookupTransform(_world_frame_, frame_id, ros::Time(0));
    } else {
      world_to_robot = this->m_buffer.lookupTransform(_world_frame_, frame_id, ros::Time::now());
    }
  }
  catch (...) {
    return false;
  }

  double robot_x = world_to_robot.transform.translation.x;
  double robot_y = world_to_robot.transform.translation.y;
  double robot_z = world_to_robot.transform.translation.z;

  bool success;

  // copy the maps
  {
    std::scoped_lock lock(mutex_octree_);

    octree->clear();

    const octomap::point3d p_min =
        octomap::point3d(float(robot_x - _local_map_distance_), float(robot_y - _local_map_distance_), float(robot_z - _local_map_distance_));
    const octomap::point3d p_max =
        octomap::point3d(float(robot_x + _local_map_distance_), float(robot_y + _local_map_distance_), float(robot_z + _local_map_distance_));

    success = copyInsideBBX(octree_, octree, p_min, p_max);
  }

  {
    std::scoped_lock lock(mutex_avg_time_local_map_processing_);

    ros::Time time_end = ros::Time::now();

    double exec_duration = (time_end - time_start).toSec();

    double coef                    = 0.95;
    avg_time_local_map_processing_ = coef * avg_time_local_map_processing_ + (1.0 - coef) * exec_duration;

    ROS_INFO_THROTTLE(5.0, "[OctomapServer]: avg local map creation time = %.3f sec", avg_time_local_map_processing_);
  }

  return success;
}

//}

}  // namespace mrs_octomap_server

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_octomap_server::OctomapServer, nodelet::Nodelet)
