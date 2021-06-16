/* includes //{ */

#include "tf2/LinearMath/Transform.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
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
#include <Eigen/Geometry>

#include <octomap_msgs/BoundingBoxQueryRequest.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>

#include <mrs_lib/scope_timer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

#include <mrs_msgs/String.h>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <mrs_octomap_server/conversions.h>

#include <laser_geometry/laser_geometry.h>

#include <ouster_ros/point.h>

#include <cmath>
#include <memory>
#include <chrono>

/* #include <ouster_ros/OSConfigSrv.h> */

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

using pt_t = ouster_ros::Point;
using pc_t = pcl::PointCloud<pt_t>;

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

  using OctomapSrv = octomap_msgs::GetOctomap;
  using BBXSrv     = octomap_msgs::BoundingBoxQuery;

  virtual void onInit();

  bool callbackLoadMap(mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& resp);
  bool callbackSaveMap(mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& resp);

  virtual bool callbackGetOctomapBinary(octomap_msgs::GetOctomapRequest& req, octomap_msgs::GetOctomapResponse& res);
  virtual bool callbackGetOctomapFull(octomap_msgs::GetOctomapRequest& req, octomap_msgs::GetOctomapResponse& res);

  bool callbackClearBBX(octomap_msgs::BoundingBoxQueryRequest& req, octomap_msgs::BoundingBoxQueryRequest& resp);
  bool callbackResetMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  virtual void insertCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual void insertLaserScanCallback(const sensor_msgs::LaserScanConstPtr& scan);
  virtual bool loadFromFile(const std::string& filename);
  virtual bool saveToFile(const std::string& filename);

protected:
  ros::NodeHandle nh_;
  bool            is_initialized_;

  // | -------------------- topic subscribers ------------------- |

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> m_pointCloudSub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>>   m_laserScanSub;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher m_occupiedPointCloudPub;
  ros::Publisher m_freePointCloudPub;
  ros::Publisher m_fmarkerPub;
  ros::Publisher m_markerPub;
  ros::Publisher m_binaryMapPub;
  ros::Publisher m_fullMapPub;
  ros::Publisher pub_octomap_local_;

  // throttled markers
  ros::Publisher pub_throttled_occupied_marker_;
  ros::Publisher pub_throttled_free_marker_;
  ros::Time      time_last_occupied_published_;
  ros::Time      time_last_free_published_;
  double         throttle_occupied_vis_ = 1;
  double         throttle_free_vis_     = 1;

  // | -------------------- service serviers -------------------- |

  ros::ServiceServer m_octomapBinaryService;
  ros::ServiceServer m_octomapFullService;
  ros::ServiceServer m_clearBBXService;
  ros::ServiceServer m_resetService;
  ros::ServiceServer ss_save_map_;
  ros::ServiceServer ss_load_map_;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_main_;
  double     _timer_main_rate_;
  void       timerMain([[maybe_unused]] const ros::TimerEvent& event);

  // | ----------------------- parameters ----------------------- |

  bool _simulation_;

  double _local_map_enabled_;
  double _local_map_size_;

  static std_msgs::ColorRGBA heightMapColor(double h);
  double                     _occupancy_cube_size_factor_;

  tf2_ros::Buffer                             m_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

  std::shared_ptr<OcTreeT> octree_;
  std::mutex               mutex_octree_;

  std::shared_ptr<OcTreeT> octree_local_;
  std::mutex               mutex_octree_local_;

  double cloud_insertion_time_ = 0;

  octomap::KeyRay    m_keyRay;  // temp storage for ray casting
  octomap::OcTreeKey m_updateBBXMin;
  octomap::OcTreeKey m_updateBBXMax;

  double              m_maxRange;
  std::string         _world_frame_;  // the map frame
  std::string         _robot_frame_;  // the map frame
  bool                m_useHeightMap;
  bool                m_useColoredMap;
  std_msgs::ColorRGBA m_color;
  std_msgs::ColorRGBA m_colorFree;
  double              m_colorFactor;
  bool                m_publishFreeSpace;
  double              m_res;
  unsigned            m_treeDepth;
  unsigned            m_maxTreeDepth;
  double              m_occupancyMinZ;
  double              m_occupancyMaxZ;
  double              m_minSizeX;
  double              m_minSizeY;
  bool                m_filterSpeckles;
  bool                m_compressMap;
  std::string         _map_path_;

  bool   m_filterGroundPlane;
  double m_ZGroundFilterDistance;

  bool   m_localMapping;
  double m_localMapDistance;

  bool   m_clearNearby;
  double m_nearbyClearingDistance;

  bool   _unknown_rays_update_free_space_;
  double _unknown_rays_distance_;

  laser_geometry::LaserProjection projector_;

  bool clearOutsideBBX(std::shared_ptr<OcTreeT> &octree, const octomap::point3d& p_min, const octomap::point3d& p_max);
  bool copyInsideBBX(std::shared_ptr<OcTreeT>& from, std::shared_ptr<OcTreeT>& to, const octomap::point3d& p_min, const octomap::point3d& p_max);
  bool clearInsideBBX(const octomap::point3d& p_min, const octomap::point3d& p_max);

  bool createLocalMap(const std::string frame_id, const double radius, std::shared_ptr<OcTreeT>& octree);

  inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
    for (unsigned i = 0; i < 3; ++i)
      min[i] = std::min(in[i], min[i]);
  };

  inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
    for (unsigned i = 0; i < 3; ++i)
      max[i] = std::max(in[i], max[i]);
  };

  /// Test if key is within update area of map (2D, ignores height)
  inline bool isInUpdateBBX(const OcTreeT::iterator& it) const {
    // 2^(tree_depth-depth) voxels wide:
    unsigned           voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
    octomap::OcTreeKey key        = it.getIndexKey();  // lower corner of voxel
    return (key[0] + voxelWidth >= m_updateBBXMin[0] && key[1] + voxelWidth >= m_updateBBXMin[1] && key[0] <= m_updateBBXMax[0] && key[1] <= m_updateBBXMax[1]);
  }

  void publishBinaryOctoMap(const ros::Time& rostime) const;
  void publishFullOctoMap(const ros::Time& rostime) const;

  void publishLocalOctomap(void);

  virtual void publishAll(void);

  /**
   * @brief update occupancy map with a scan
   * The scans should be in the global map frame.
   *
   * @param sensorOrigin origin of the measurements for raycasting
   * @param cloud
   * @param free_cloud
   */
  virtual void insertPointCloud(const geometry_msgs::Vector3& sensorOrigin, const PCLPointCloud::ConstPtr& cloud, const PCLPointCloud::ConstPtr& free_cloud);

  /**
   * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
   * @param key
   * @return
   */
  bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  // hook that is called when traversing all nodes of the updated Octree (does nothing here)
  virtual void handleNode([[maybe_unused]] const OcTreeT::iterator& it){};

  // hook that is called when traversing all nodes of the updated Octree in the updated area (does nothing here)
  virtual void handleNodeInBBX([[maybe_unused]] const OcTreeT::iterator& it){};

  // --------------------------------------------------------------
  // |                  Sensor-related variables                  |
  // --------------------------------------------------------------

  void initializeOusterLUT(const size_t w, const size_t h, const std::vector<double>& azimuth_angles_deg, const std::vector<double>& altitude_angles_deg,
                           const double range_unit = 0.001, const double lidar_origin_to_beam_origin_mm = 0.0,
                           const ouster::mat4d& tf = ouster::mat4d::Identity());

  void initializeLidarLUTSimulation(const size_t w, const size_t h);

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

  mrs_lib::ParamLoader pl(nh_, ros::this_node::getName());

  /* load params //{ */
  ROS_INFO("[%s]: -------------- Loading parameters --------------", ros::this_node::getName().c_str());

  pl.loadParam("simulation", _simulation_);

  pl.loadParam("main_timer/rate", _timer_main_rate_);

  pl.loadParam("resolution", m_res, 0.4);
  pl.loadParam("world_frame_id", _world_frame_);
  pl.loadParam("robot_frame_id", _robot_frame_);
  pl.loadParam("compress_map", m_compressMap, true);

  pl.loadParam("map_path", _map_path_);

  pl.loadParam("unknown_rays/update_free_space", _unknown_rays_update_free_space_);
  pl.loadParam("unknown_rays/ray_distance", _unknown_rays_distance_);

  pl.loadParam("sensor_params_3d/enabled", m_sensor_3d_params_enabled);
  pl.loadParam("sensor_params_3d/vertical_fov_angle", m_sensor_3d_vfov);
  pl.loadParam("sensor_params_3d/vertical_rays", m_sensor_3d_vrays);
  pl.loadParam("sensor_params_3d/horizontal_rays", m_sensor_3d_hrays);

  double probHit, probMiss, thresMin, thresMax;
  pl.loadParam("sensor_model/hit", probHit);
  pl.loadParam("sensor_model/miss", probMiss);
  pl.loadParam("sensor_model/min", thresMin);
  pl.loadParam("sensor_model/max", thresMax);
  pl.loadParam("sensor_model/max_range", m_maxRange);

  pl.loadParam("local_mapping/enable", m_localMapping, false);
  pl.loadParam("local_mapping/distance", m_localMapDistance, 10.0);

  pl.loadParam("nearby_clearing/enable", m_clearNearby, false);
  pl.loadParam("nearby_clearing/distance", m_nearbyClearingDistance, 0.3);

  pl.loadParam("visualization/occupied_throttled", throttle_occupied_vis_);
  pl.loadParam("visualization/free_throttled", throttle_free_vis_);

  pl.loadParam("visualization/occupancy/min_z", m_occupancyMinZ);
  pl.loadParam("visualization/occupancy/max_z", m_occupancyMaxZ);
  pl.loadParam("visualization/occupancy/cube_size_factor", _occupancy_cube_size_factor_);

  pl.loadParam("visualization/colored_map/enabled", m_useColoredMap, false);

  pl.loadParam("visualization/height_map/enabled", m_useHeightMap, true);
  pl.loadParam("visualization/height_map/color_factor", m_colorFactor, 0.8);

  double r, g, b, a;
  pl.loadParam("visualization/height_map/color/r", r);
  pl.loadParam("visualization/height_map/color/g", g);
  pl.loadParam("visualization/height_map/color/b", b);
  pl.loadParam("visualization/height_map/color/a", a);
  m_color.r = r;
  m_color.g = g;
  m_color.b = b;
  m_color.a = a;

  pl.loadParam("visualization/publish_free_space", m_publishFreeSpace, false);
  pl.loadParam("visualization/color_free/r", r);
  pl.loadParam("visualization/color_free/g", g);
  pl.loadParam("visualization/color_free/b", b);
  pl.loadParam("visualization/color_free/a", a);
  m_colorFree.r = r;
  m_colorFree.g = g;
  m_colorFree.b = b;
  m_colorFree.a = a;

  if (!pl.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", ros::this_node::getName().c_str());
    ros::requestShutdown();
  }

  //}

  /* check params //{ */

  if (m_useHeightMap && m_useColoredMap) {
    std::string msg = std::string("You enabled both height map and RGBcolor registration.") + " This is contradictory. " + "Defaulting to height map.";
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.c_str());
    m_useColoredMap = false;
  }

  if (m_useColoredMap) {
#ifdef COLOR_OCTOMAP_SERVER
    ROS_WARN("[%s]: Using RGB color registration (if information available)", ros::this_node::getName().c_str());
#else
    std::string msg = std::string("Colored map requested in launch file") + " - node not running/compiled to support colors, " +
                      "please define COLOR_OCTOMAP_SERVER and recompile or launch " + "the octomap_color_server node";
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.c_str());
#endif
  }

  if (m_localMapping && m_localMapDistance < m_maxRange) {
    std::string msg = std::string("You enabled using only the local map. ") +
                      "However, the local distance for the map is lower than the maximal sensor range. " +
                      "Defaulting the local distance for the map to the maximal sensor range.";
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.c_str());
    m_localMapDistance = m_maxRange;
  }

  //}

  /* initialize sensor LUT model //{ */

  if (m_sensor_3d_params_enabled) {

    if (_simulation_) {

      initializeLidarLUTSimulation(m_sensor_3d_hrays, m_sensor_3d_vrays);

      ROS_INFO("[OctomapServer]: Ouster LUT model initialized (simulation)");

    } else {

      initializeOusterLUT(m_sensor_3d_hrays, m_sensor_3d_vrays, ouster::sensor::gen1_azimuth_angles, ouster::sensor::gen1_altitude_angles);

      ROS_INFO("[OctomapServer]: Ouster LUT model initialized (real hw)");
    }
  }

  //}

  /* initialize octomap object & params //{ */

  octree_ = std::make_shared<OcTreeT>(m_res);
  octree_->setProbHit(probHit);
  octree_->setProbMiss(probMiss);
  octree_->setClampingThresMin(thresMin);
  octree_->setClampingThresMax(thresMax);
  m_treeDepth    = octree_->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;

  //}

  /* tf_listener //{ */

  this->m_buffer.setUsingDedicatedThread(true);
  this->m_tfListener = std::make_unique<tf2_ros::TransformListener>(m_buffer, ros::this_node::getName());

  //}

  /* publishers //{ */

  this->m_markerPub             = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array_out", 1);
  this->m_binaryMapPub          = nh_.advertise<octomap_msgs::Octomap>("octomap_binary_out", 1);
  this->m_fullMapPub            = nh_.advertise<octomap_msgs::Octomap>("octomap_full_out", 1);
  this->m_occupiedPointCloudPub = nh_.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers_out", 1);
  this->m_freePointCloudPub     = nh_.advertise<sensor_msgs::PointCloud2>("octomap_free_centers_out", 1);
  this->m_fmarkerPub            = nh_.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array_out", 1);
  pub_octomap_local_            = nh_.advertise<octomap_msgs::Octomap>("octomap_local_out", 1);

  // throttled
  this->pub_throttled_free_marker_     = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array_throttled_out", 1);
  this->pub_throttled_occupied_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array_throttled_out", 1);

  time_last_occupied_published_ = ros::Time(0);
  time_last_free_published_     = ros::Time(0);

  //}

  /* subscribers //{ */

  // Point Cloud

  this->m_pointCloudSub = std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, "point_cloud_in", 5);
  this->m_pointCloudSub->registerCallback(std::bind(&OctomapServer::insertCloudCallback, this, ph::_1));

  // Laser scan
  this->m_laserScanSub = std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, "laser_scan_in", 5);
  this->m_laserScanSub->registerCallback(std::bind(&OctomapServer::insertLaserScanCallback, this, ph::_1));

  //}

  /* service servers //{ */

  this->m_octomapBinaryService = nh_.advertiseService("octomap_binary", &OctomapServer::callbackGetOctomapBinary, this);
  this->m_octomapFullService   = nh_.advertiseService("octomap_full", &OctomapServer::callbackGetOctomapFull, this);
  this->m_clearBBXService      = nh_.advertiseService("clear_bbx", &OctomapServer::callbackClearBBX, this);
  this->m_resetService         = nh_.advertiseService("reset", &OctomapServer::callbackResetMap, this);
  this->ss_save_map_           = nh_.advertiseService("save_map_in", &OctomapServer::callbackSaveMap, this);
  this->ss_load_map_           = nh_.advertiseService("load_map_in", &OctomapServer::callbackLoadMap, this);

  //}

  /* timers //{ */

  timer_main_ = nh_.createTimer(ros::Rate(_timer_main_rate_), &OctomapServer::timerMain, this);

  //}

  is_initialized_ = true;

  ROS_INFO("[%s]: Initialized", ros::this_node::getName().c_str());
}

//}

// | --------------------- topic callbacks -------------------- |

/* OctomapServer::insertLaserScanCallback() //{ */

void OctomapServer::insertLaserScanCallback(const sensor_msgs::LaserScanConstPtr& scan) {

  /* mrs_lib::ScopeTimer scope_timer("insertLaserScanCallback"); */

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

  /* scope_timer.checkpoint("transform"); */

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

  /* scope_timer.checkpoint("insertPointCloud"); */

  insertPointCloud(sensorToWorldTf.transform.translation, pc, free_vectors_pc);

  const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation);
}

//}

/* OctomapServer::insertCloudCallback() //{ */

void OctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud) {

  /* mrs_lib::ScopeTimer scope_timer("insertCloudCallback"); */

  if (!is_initialized_) {
    return;
  }

  ros::Time time_start = ros::Time::now();

  PCLPointCloud::Ptr pc              = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = boost::make_shared<PCLPointCloud>();
  pcl::fromROSMsg(*cloud, *pc);

  Eigen::Matrix4f                 sensorToWorld;
  geometry_msgs::TransformStamped sensorToWorldTf;

  /* scope_timer.checkpoint("transform"); */

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

  /* scope_timer.checkpoint("insertPointCloud"); */

  insertPointCloud(sensorToWorldTf.transform.translation, pc, free_vectors_pc);

  const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation);

  /* scope_timer.checkpoint("publish"); */

  {
    ros::Time time_end = ros::Time::now();

    double exec_duration = (time_end - time_start).toSec();

    double coef           = 0.95;
    cloud_insertion_time_ = coef * cloud_insertion_time_ + (1.0 - coef) * exec_duration;

    ROS_INFO_THROTTLE(5.0, "[OctomapServer]: avg cloud insertion time = %.3f sec", cloud_insertion_time_);
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* OctomapServer::callbackLoadMap() //{ */

bool OctomapServer::callbackLoadMap([[maybe_unused]] mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  ROS_INFO("[OctomapServer]: loading map");

  bool success = loadFromFile(_map_path_ + "/" + req.value + ".ot");

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

/* OctomapServer::callbackSaveMap() //{ */

bool OctomapServer::callbackSaveMap([[maybe_unused]] mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  std::scoped_lock lock(mutex_octree_);

  std::string mapname = _map_path_ + "/" + req.value + ".ot";

  ROS_INFO("Map received (%zu nodes, %f m res), saving to %s", octree_->size(), octree_->getResolution(), mapname.c_str());

  std::string suffix = mapname.substr(mapname.length() - 3, 3);

  if (!octree_->write(mapname)) {
    ROS_ERROR("Error writing to file %s", mapname.c_str());
  }

  res.message = "saved";
  res.success = true;

  return true;
}

//}

/* OctomapServer::callbackResetMap() //{ */

bool OctomapServer::callbackResetMap([[maybe_unused]] std_srvs::Empty::Request& req, [[maybe_unused]] std_srvs::Empty::Response& resp) {

  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_treeDepth + 1);
  auto rostime = ros::Time::now();

  octree_->clear();

  ROS_INFO("[OctomapServer]: octomap cleared");

  publishBinaryOctoMap(rostime);

  for (size_t i = 0; i < occupiedNodesVis.markers.size(); ++i) {
    occupiedNodesVis.markers[i].header.frame_id = _world_frame_;
    occupiedNodesVis.markers[i].header.stamp    = rostime;
    occupiedNodesVis.markers[i].ns              = "map";
    occupiedNodesVis.markers[i].id              = i;
    occupiedNodesVis.markers[i].type            = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].action          = visualization_msgs::Marker::DELETE;
  }

  m_markerPub.publish(occupiedNodesVis);

  visualization_msgs::MarkerArray freeNodesVis;

  freeNodesVis.markers.resize(m_treeDepth + 1);

  for (size_t i = 0; i < freeNodesVis.markers.size(); ++i) {
    freeNodesVis.markers[i].header.frame_id = _world_frame_;
    freeNodesVis.markers[i].header.stamp    = rostime;
    freeNodesVis.markers[i].ns              = "map";
    freeNodesVis.markers[i].id              = i;
    freeNodesVis.markers[i].type            = visualization_msgs::Marker::CUBE_LIST;
    freeNodesVis.markers[i].action          = visualization_msgs::Marker::DELETE;
  }

  m_fmarkerPub.publish(freeNodesVis);

  return true;
}

//}

/* OctomapServer::callbackClearBBX() //{ */

bool OctomapServer::callbackClearBBX(octomap_msgs::BoundingBoxQueryRequest& req, [[maybe_unused]] octomap_msgs::BoundingBoxQueryRequest& resp) {

  octomap::point3d min = octomap::pointMsgToOctomap(req.min);
  octomap::point3d max = octomap::pointMsgToOctomap(req.max);

  double thresMin = octree_->getClampingThresMin();

  for (auto it = octree_->begin_leafs_bbx(min, max), end = octree_->end_leafs_bbx(); it != end; ++it) {
    it->setLogOdds(octomap::logodds(thresMin));
  }

  octree_->updateInnerOccupancy();

  return true;
}

//}

/* OctomapServer::callbackGetOctomapBinary() //{ */

bool OctomapServer::callbackGetOctomapBinary([[maybe_unused]] octomap_msgs::GetOctomapRequest& req, octomap_msgs::GetOctomapResponse& res) {

  ROS_INFO("[%s]: Sending binary map data on service request", ros::this_node::getName().c_str());

  res.map.header.frame_id = _world_frame_;
  res.map.header.stamp    = ros::Time::now();

  if (!octomap_msgs::binaryMapToMsg(*octree_, res.map)) {
    return false;
  }

  return true;
}

//}

/* OctomapServer::callbackGetOctomapFull() //{ */

bool OctomapServer::callbackGetOctomapFull([[maybe_unused]] octomap_msgs::GetOctomapRequest& req, octomap_msgs::GetOctomapResponse& res) {

  ROS_INFO("[%s]: Sending full map data on service request", ros::this_node::getName().c_str());

  res.map.header.frame_id = _world_frame_;
  res.map.header.stamp    = ros::Time::now();

  if (!octomap_msgs::fullMapToMsg(*octree_, res.map)) {
    return false;
  }

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void OctomapServer::timerMain([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: main timer spinning");

  publishAll();

  // | ---------------- postprocess the local map --------------- |

  if (m_localMapping) {

    std::scoped_lock lock(mutex_octree_local_);

    bool success = createLocalMap(_robot_frame_, m_localMapDistance, octree_local_);

    if (success) {

      publishLocalOctomap();
    }
  }
}

//}

// | ------------------------ routines ------------------------ |

/* OctomapServer::insertPointCloud() //{ */

void OctomapServer::insertPointCloud(const geometry_msgs::Vector3& sensorOriginTf, const PCLPointCloud::ConstPtr& cloud,
                                     const PCLPointCloud::ConstPtr& free_vectors_cloud) {

  std::scoped_lock lock(mutex_octree_);

  /* mrs_lib::ScopeTimer scope_timer("insertPointCloud"); */

  const octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

  if (!octree_->coordToKeyChecked(sensorOrigin, m_updateBBXMin) || !octree_->coordToKeyChecked(sensorOrigin, m_updateBBXMax)) {
    ROS_ERROR_STREAM("Could not generate Key for origin " << sensorOrigin);
  }

  const float free_space_ray_len = 15.0;

  // instead of direct scan insertion, compute probabilistic update
  octomap::KeySet free_cells, occupied_cells;
  const bool      free_space_bounded = free_space_ray_len > 0.0f;

  /* scope_timer.checkpoint("sortingThroughPoints"); */

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

  /* scope_timer.checkpoint("markingFree"); */

  // mark free cells only if not seen occupied in this cloud
  for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
    if (occupied_cells.find(*it) == occupied_cells.end()) {
      octree_->updateNode(*it, false);
    }
  }

  /* scope_timer.checkpoint("markingOcuppied"); */

  // now mark all occupied cells:
  for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {
    octree_->updateNode(*it, true);
  }

  /* scope_timer.checkpoint("compresssing"); */

  if (m_compressMap) {
    octree_->prune();
  }
}

//}

/* initializeLidarLUT() //{ */

void OctomapServer::initializeOusterLUT(const size_t w, const size_t h, const std::vector<double>& azimuth_angles_deg,
                                        const std::vector<double>& altitude_angles_deg, const double range_unit, const double lidar_origin_to_beam_origin_mm,
                                        const ouster::mat4d& tf) {

  ouster::XYZLut xyz_lut;

  xyz_lut = ouster::make_xyz_lut(w, h, range_unit, lidar_origin_to_beam_origin_mm, tf, azimuth_angles_deg, altitude_angles_deg);

  if (xyz_lut.direction.cols() != xyz_lut.offset.cols()) {
    ROS_ERROR_STREAM("[TODO]: XYZ LUT doesn't have the correct number of elements (number of direction vectors "
                     << xyz_lut.direction.cols() << " is not equal to the number of offset vectors " << xyz_lut.offset.cols() << ")!");
  }

  m_sensor_3d_xyz_lut = {xyz_lut.direction.cast<float>().transpose(), xyz_lut.offset.cast<float>().transpose()};
  m_sensor_3d_xyz_lut.directions.colwise().normalize();

  ROS_INFO_STREAM("[TODO]: Initialized XYZ LUT table with " << m_sensor_3d_xyz_lut.directions.cols() << " elements.");
}

//}

/* initializeLidarLUTSimulation() //{ */

// copied directly from the simulation plugin
void OctomapServer::initializeLidarLUTSimulation(const size_t w, const size_t h) {

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

/* OctomapServer::loadFromFile() //{ */

bool OctomapServer::loadFromFile(const std::string& filename) {

  {
    std::scoped_lock lock(mutex_octree_);

    if (filename.length() <= 3)
      return false;

    std::string suffix = filename.substr(filename.length() - 3, 3);

    if (suffix == ".bt") {
      if (!octree_->readBinary(filename)) {
        return false;
      }
    } else if (suffix == ".ot") {

      auto tree = octomap::AbstractOcTree::read(filename);
      if (!tree) {
        return false;
      }

      OcTreeT* octree = dynamic_cast<OcTreeT*>(tree);
      octree_         = std::shared_ptr<OcTreeT>(octree);

      if (!octree_) {
        std::string msg = "Could not read OcTree in file";
        ROS_INFO("[%s]: %s", ros::this_node::getName().c_str(), msg.c_str());
        return false;
      }

    } else {
      return false;
    }

    ROS_INFO("[%s]: Octomap file %s loaded (%zu nodes).", ros::this_node::getName().c_str(), filename.c_str(), octree_->size());

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

/* OctomapServer::saveToFile() //{ */

bool OctomapServer::saveToFile(const std::string& filename) {

  if (filename.length() <= 3)
    return false;

  std::string suffix = filename.substr(filename.length() - 3, 3);

  if (suffix == ".bt") {
    if (!octree_->readBinary(filename)) {
      return false;
    }
  } else if (suffix == ".ot") {

    auto tree = octomap::AbstractOcTree::read(filename);
    if (!tree) {
      return false;
    }

    OcTreeT* octree = dynamic_cast<OcTreeT*>(tree);
    octree_         = std::shared_ptr<OcTreeT>(octree);

    if (!octree_) {
      std::string msg = "Could not read OcTree in file";
      ROS_INFO("[%s]: %s", ros::this_node::getName().c_str(), msg.c_str());
      return false;
    }

  } else {
    return false;
  }

  ROS_INFO("[%s]: Octomap file %s loaded (%zu nodes).", ros::this_node::getName().c_str(), filename.c_str(), octree_->size());

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

  return true;
}

//}

/* OctomapServer::publishAll() //{ */

void OctomapServer::publishAll(void) {

  std::scoped_lock lock(mutex_octree_);

  // ros::WallTime startTime = ros::WallTime::now();

  size_t octomap_size = octree_->size();

  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomap_size <= 1) {
    ROS_WARN("[%s]: Nothing to publish, octree is empty", ros::this_node::getName().c_str());
    return;
  }

  bool publishFreeMarkerArray = m_publishFreeSpace && (m_fmarkerPub.getNumSubscribers() > 0 || pub_throttled_occupied_marker_.getNumSubscribers() > 0);
  bool publishMarkerArray     = (m_markerPub.getNumSubscribers() > 0 || pub_throttled_free_marker_.getNumSubscribers());
  bool publishPointCloud      = m_occupiedPointCloudPub.getNumSubscribers() > 0 || m_freePointCloudPub.getNumSubscribers() > 0;
  bool publishBinaryMap       = m_binaryMapPub.getNumSubscribers() > 0;
  bool publishFullMap         = m_fullMapPub.getNumSubscribers() > 0;

  // init markers for free space:
  visualization_msgs::MarkerArray freeNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  freeNodesVis.markers.resize(m_treeDepth + 1);

  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, 0.0);
  geometry_msgs::Pose pose;
  pose.orientation = tf2::toMsg(quaternion);

  // init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth + 1);

  // init pointcloud:
  pcl::PointCloud<PCLPoint> occupied_pclCloud;
  pcl::PointCloud<PCLPoint> free_pclCloud;

  // now, traverse all leafs in the tree:
  for (auto it = octree_->begin(m_maxTreeDepth), end = octree_->end(); it != end; ++it) {

    bool inUpdateBBX = isInUpdateBBX(it);

    // call general hook:
    handleNode(it);

    if (inUpdateBBX) {
      handleNodeInBBX(it);
    }

    if (octree_->isNodeOccupied(*it)) {
      double z         = it.getZ();
      double half_size = it.getSize() / 2.0;

      if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {
        double x = it.getX();
        double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
        int r = it->getColor().r;
        int g = it->getColor().g;
        int b = it->getColor().b;
#endif

        // create marker:
        if (publishMarkerArray) {

          unsigned idx = it.getDepth();
          assert(idx < occupiedNodesVis.markers.size());

          geometry_msgs::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;

          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);

          if (m_useHeightMap) {
            double minX, minY, minZ, maxX, maxY, maxZ;
            octree_->getMetricMin(minX, minY, minZ);
            octree_->getMetricMax(maxX, maxY, maxZ);

            double h = (1.0 - std::min(std::max((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;
            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
          }

#ifdef COLOR_OCTOMAP_SERVER
          if (m_useColoredMap) {
            // TODO
            // potentially use occupancy as measure for alpha channel?
            std_msgs::msg::ColorRGBA _color;
            _color.r = (r / 255.);
            _color.g = (g / 255.);
            _color.b = (b / 255.);
            _color.a = 1.0;
            occupiedNodesVis.markers[idx].colors.push_back(_color);
          }
#endif
        }

        // insert into pointcloud:
        if (publishPointCloud) {

#ifdef COLOR_OCTOMAP_SERVER
          PCLPoint _point = PCLPoint();
          _point.x        = x;
          _point.y        = y;
          _point.z        = z;
          _point.r        = r;
          _point.g        = g;
          _point.b        = b;
          occupied_pclCloud.push_back(_point);
#else
          occupied_pclCloud.push_back(PCLPoint(float(x), float(y), float(z)));
#endif
        }
      }
    } else {
      // node not occupied => mark as free in 2D map if unknown so far
      double z         = it.getZ();
      double half_size = it.getSize() / 2.0;

      if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {

        if (m_publishFreeSpace) {
          double x = it.getX();
          double y = it.getY();

          // create marker for free space:
          if (publishFreeMarkerArray) {

            unsigned idx = it.getDepth();
            assert(idx < freeNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[idx].points.push_back(cubeCenter);
          }
        }

        // insert into pointcloud:
        if (publishPointCloud) {
          double x = it.getX();
          double y = it.getY();
          free_pclCloud.push_back(PCLPoint(float(x), float(y), float(z)));
        }
      }
    }
  }

  // finish MarkerArray:
  if (publishMarkerArray) {

    for (size_t i = 0; i < occupiedNodesVis.markers.size(); ++i) {

      double size = octree_->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = _world_frame_;
      occupiedNodesVis.markers[i].header.stamp    = ros::Time::now();
      occupiedNodesVis.markers[i].ns              = "map";
      occupiedNodesVis.markers[i].id              = i;
      occupiedNodesVis.markers[i].type            = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x         = size * _occupancy_cube_size_factor_;
      occupiedNodesVis.markers[i].scale.y         = size * _occupancy_cube_size_factor_;
      occupiedNodesVis.markers[i].scale.z         = size * _occupancy_cube_size_factor_;

      if (!m_useColoredMap)
        occupiedNodesVis.markers[i].color = m_color;


      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_markerPub.publish(occupiedNodesVis);

    // publisher throttled
    {
      const double last_pub_time = (ros::Time::now() - time_last_occupied_published_).toSec();
      const double max_time      = 1.0 / throttle_occupied_vis_;

      if (last_pub_time >= max_time) {
        pub_throttled_free_marker_.publish(occupiedNodesVis);
        time_last_occupied_published_ = ros::Time::now();
      }
    }
  }

  // finish FreeMarkerArray:
  if (publishFreeMarkerArray) {

    for (size_t i = 0; i < freeNodesVis.markers.size(); ++i) {
      double size = octree_->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = _world_frame_;
      freeNodesVis.markers[i].header.stamp    = ros::Time::now();
      freeNodesVis.markers[i].ns              = "map";
      freeNodesVis.markers[i].id              = i;
      freeNodesVis.markers[i].type            = visualization_msgs::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x         = size * _occupancy_cube_size_factor_;
      freeNodesVis.markers[i].scale.y         = size * _occupancy_cube_size_factor_;
      freeNodesVis.markers[i].scale.z         = size * _occupancy_cube_size_factor_;
      freeNodesVis.markers[i].color           = m_colorFree;

      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_fmarkerPub.publish(freeNodesVis);

    // publisher throttled
    {
      const double last_pub_time = (ros::Time::now() - time_last_free_published_).toSec();
      const double max_time      = 1.0 / throttle_free_vis_;

      if (last_pub_time >= max_time) {
        pub_throttled_occupied_marker_.publish(freeNodesVis);
        time_last_free_published_ = ros::Time::now();
      }
    }
  }

  // finish pointcloud:
  if (publishPointCloud) {

    sensor_msgs::PointCloud2 cloud;
    // occupied
    pcl::toROSMsg(occupied_pclCloud, cloud);
    cloud.header.frame_id = _world_frame_;
    cloud.header.stamp    = ros::Time::now();
    m_occupiedPointCloudPub.publish(cloud);

    // free
    pcl::toROSMsg(free_pclCloud, cloud);
    cloud.header.frame_id = _world_frame_;
    cloud.header.stamp    = ros::Time::now();
    m_freePointCloudPub.publish(cloud);
  }

  if (publishBinaryMap) {
    publishBinaryOctoMap(ros::Time::now());
  }

  if (publishFullMap) {
    publishFullOctoMap(ros::Time::now());
  }

  /*
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);
  */
}

//}

/* OctomapServer::heightMapColor() //{ */

std_msgs::ColorRGBA OctomapServer::heightMapColor(double h) {

  std_msgs::ColorRGBA color;

  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int    i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:
      color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:
      color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:
      color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:
      color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:
      color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:
      color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
}

//}

/*  OctomapServer::publishFullOctoMap() //{ */

void OctomapServer::publishFullOctoMap(const ros::Time& rostime) const {

  octomap_msgs::Octomap map;
  map.header.frame_id = _world_frame_;
  map.header.stamp    = rostime;

  if (octomap_msgs::fullMapToMsg(*octree_, map)) {
    m_fullMapPub.publish(map);
  } else {
    ROS_ERROR("[%s]: Error serializing OctoMap", ros::this_node::getName().c_str());
  }
}

//}

/*  OctomapServer::publishLocalOctomap() //{ */

void OctomapServer::publishLocalOctomap(void) {

  ROS_INFO_ONCE("[OctomapServer]: publishing local octomap");

  octomap_msgs::Octomap map;
  map.header.frame_id = _world_frame_;
  map.header.stamp    = ros::Time::now();

  if (octomap_msgs::fullMapToMsg(*octree_local_, map)) {
    pub_octomap_local_.publish(map);
  } else {
    ROS_ERROR("[%s]: Error serializing local OctoMap", ros::this_node::getName().c_str());
  }
}

//}

/* OctomapServer::publishBinaryOctoMap() //{ */

void OctomapServer::publishBinaryOctoMap(const ros::Time& rostime) const {

  octomap_msgs::Octomap map;
  map.header.frame_id = _world_frame_;
  map.header.stamp    = rostime;

  if (octomap_msgs::binaryMapToMsg(*octree_, map)) {
    m_binaryMapPub.publish(map);
  } else {
    ROS_ERROR("[%s]: Error serializing OctoMap", ros::this_node::getName().c_str());
  }
}

//}

/* OctomapServer::clearOutsideBBX() //{ */

bool OctomapServer::clearOutsideBBX(std::shared_ptr<OcTreeT>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey minKey, maxKey;

  if (!octree->coordToKeyChecked(p_min, minKey) || !octree->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  std::vector<std::pair<octomap::OcTreeKey, unsigned int>> keys;

  for (OcTreeT::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {

    // check if outside of bbx:
    octomap::OcTreeKey k = it.getKey();

    if (k[0] < minKey[0] || k[1] < minKey[1] || k[2] < minKey[2] || k[0] > maxKey[0] || k[1] > maxKey[1] || k[2] > maxKey[2]) {
      keys.push_back(std::make_pair(k, it.getDepth()));
    }
  }

  for (auto k : keys) {
    octree->deleteNode(k.first, k.second);
  }

  ROS_INFO("[%s]: Number of voxels removed outside local area: %ld", ros::this_node::getName().c_str(), keys.size());

  return true;
}

//}

/* OctomapServer::copyInsideBBX() //{ */

bool OctomapServer::copyInsideBBX(std::shared_ptr<OcTreeT>& from, std::shared_ptr<OcTreeT>& to, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey minKey, maxKey;

  if (!from->coordToKeyChecked(p_min, minKey) || !from->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  std::vector<std::pair<octomap::OcTreeKey, unsigned int>> keys;

  for (OcTreeT::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max), end = from->end_leafs_bbx(); it != end; ++it) {

    // check if outside of bbx:
    octomap::OcTreeKey k = it.getKey();

    /* if (k[0] >= minKey[0] && k[1] >= minKey[1] && k[2] >= minKey[2] && k[0] <= maxKey[0] && k[1] <= maxKey[1] && k[2] <= maxKey[2]) { */
      keys.push_back(std::make_pair(k, it.getDepth()));
    /* } */
  }

  for (auto k : keys) {

    auto node = from->search(k.first);
    to->setNodeValue(k.first, node->getValue());
    /* m_octree->updateNode(k.first, false); */
  }

  return true;
}

//}

/* OctomapServer::clearInsideBBX() //{ */

bool OctomapServer::clearInsideBBX(const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey minKey, maxKey;

  if (!octree_->coordToKeyChecked(p_min, minKey) || !octree_->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  std::vector<std::pair<octomap::OcTreeKey, unsigned int>> keys;

  for (OcTreeT::leaf_iterator it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {

    // check if outside of bbx:
    octomap::OcTreeKey k = it.getKey();

    if (k[0] >= minKey[0] && k[1] >= minKey[1] && k[2] >= minKey[2] && k[0] <= maxKey[0] && k[1] <= maxKey[1] && k[2] <= maxKey[2]) {
      keys.push_back(std::make_pair(k, it.getDepth()));
    }
  }

  for (auto k : keys) {
    octree_->setNodeValue(k.first, -1.0);
    /* m_octree->updateNode(k.first, false); */
  }

  return true;
}

//}

/* createLocalMap() //{ */

bool OctomapServer::createLocalMap(const std::string frame_id, const double radius, std::shared_ptr<OcTreeT>& octree) {

  ROS_INFO_THROTTLE(1.0, "[OctomapServer]: creating the local map");

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

  // copy the maps
  {
    std::scoped_lock lock(mutex_octree_);

    octree = std::make_shared<OcTreeT>(*octree_);
    octree->clear();

    // clear the surroundings of the robot

    const octomap::point3d p_min =
        octomap::point3d(float(robot_x - m_localMapDistance), float(robot_y - m_localMapDistance), float(robot_z - m_localMapDistance));
    const octomap::point3d p_max =
        octomap::point3d(float(robot_x + m_localMapDistance), float(robot_y + m_localMapDistance), float(robot_z + m_localMapDistance));

    if (copyInsideBBX(octree_, octree, p_min, p_max)) {
      return true;
    } else {
      return false;
    }
  }
}

//}

}  // namespace mrs_octomap_server

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_octomap_server::OctomapServer, nodelet::Nodelet)
