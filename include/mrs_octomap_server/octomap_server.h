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
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>

#include <eigen3/Eigen/Eigen>

#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
#include <mrs_lib/mutex.h>
#include <mrs_lib/scope_timer.h>

#include <mrs_octomap_tools/octomap_methods.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>

#include <mrs_msgs/SetInt.h>

#include <filesystem>

#include <mrs_octomap_server/conversions.h>

#include <laser_geometry/laser_geometry.h>

#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <mrs_octomap_server/PoseWithSize.h>


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


typedef struct
{
  double max_range;
  double free_ray_distance;
  double vertical_fov;
  int    vertical_rays;
  int    horizontal_rays;
  bool   update_free_space;
  bool   clear_occupied;
  double free_ray_distance_unknown;
} SensorParams3DLidar_t;



#ifdef COLOR_OCTOMAP_SERVER
using PCLPoint      = pcl::PointXYZRGB;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTree_t      = octomap::ColorOcTree;
#else
using PCLPoint      = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTree_t      = octomap::OcTree;
#endif

typedef enum
{

  LIDAR_3D,
  LIDAR_2D,
  LIDAR_1D,
  DEPTH_CAMERA,
  ULTRASOUND,

} SensorType_t;

const std::string _sensor_names_[] = {"LIDAR_3D", "LIDAR_2D", "LIDAR_1D", "DEPTH_CAMERA", "ULTRASOUND"};

//}


/* class OctomapServer //{ */

class OctomapServer : public nodelet::Nodelet {

public:
  virtual void onInit();



  void callback3dLidarCloud2(const sensor_msgs::PointCloud2::ConstPtr msg, const SensorType_t sensor_type, const int sensor_id, const std::string topic,
                             const bool pcl_over_max_range = false);

  void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr msg);
  void callbackCameraInfo(const sensor_msgs::CameraInfo::ConstPtr msg, const int sensor_id);

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_ = false;

  // | -------------------- topic subscribers ------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>            sh_height_;
  mrs_lib::SubscribeHandler<mrs_octomap_server::PoseWithSize>    sh_clear_box_;

  std::vector<mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>> sh_3dlaser_pc2_;
  

  // | ----------------------- publishers ----------------------- |

  ros::Publisher pub_map_global_full_;
  ros::Publisher pub_map_global_binary_;
  ros::Publisher  global_mapPub;
  ros::Publisher pub_map_local_full_;
  ros::Publisher pub_map_local_binary_;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_global_map_publisher_;
  double     _global_map_publisher_rate_;
  void       timerGlobalMapPublisher([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_global_map_creator_;
  double     _global_map_creator_rate_;
  void       timerGlobalMapCreator([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_local_map_publisher_;
  void       timerLocalMapPublisher([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_local_map_resizer_;
  void       timerLocalMapResizer([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_persistency_;
  void       timerPersistency([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_altitude_alignment_;
  void       timerAltitudeAlignment([[maybe_unused]] const ros::TimerEvent& event);

  // | ----------------------- parameters ----------------------- |

  bool        _simulation_;
  std::string _uav_name_;

  bool _scope_timer_enabled_;

  double _robot_height_;

  bool        _persistency_enabled_;
  std::string _persistency_map_name_;
  double      _persistency_save_time_;

  bool   _persistency_align_altitude_enabled_;
  double _persistency_align_altitude_distance_;

  bool _global_map_publish_full_;
  bool _global_map_publish_binary_;
  bool _global_map_enabled_;

  bool _map_while_grounded_;

  bool _local_map_publish_full_ = false;
  bool _local_map_publish_binary_ = false;

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  std::shared_ptr<OcTree_t> octree_global_;
  std::mutex                mutex_octree_global_;

  std::shared_ptr<OcTree_t> octree_local_;
  std::shared_ptr<OcTree_t> octree_local_0_;
  std::shared_ptr<OcTree_t> octree_local_1_;
  int                       octree_local_idx_ = 0;
  std::mutex                mutex_octree_local_;

  std::atomic<bool> octrees_initialized_ = false;

  double     avg_time_cloud_insertion_ = 0;
  std::mutex mutex_avg_time_cloud_insertion_;

  std::string _world_frame_;
  std::string _local_frame_;
  std::string _robot_frame_;
  double      octree_resolution_;
  bool        _global_map_compress_;
  std::string _map_path_;

  float      _local_map_width_max_;
  float      _local_map_width_min_;
  float      _local_map_height_max_;
  float      _local_map_height_min_;
  float      local_map_width_;
  float      local_map_height_;
  std::mutex mutex_local_map_dimensions_;
  double     _local_map_publisher_rate_;

  double     local_map_duty_                 = 0;
  double     _local_map_duty_high_threshold_ = 0;
  double     _local_map_duty_low_threshold_  = 0;
  std::mutex mutex_local_map_duty_;

  bool   _unknown_rays_update_free_space_;
  bool   _unknown_rays_clear_occupied_;
  double _unknown_rays_distance_;

  laser_geometry::LaserProjection projector_;

  bool copyInsideBBX2(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min, const octomap::point3d& p_max);

  bool copyLocalMap(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to);

  octomap::OcTreeNode* touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key, unsigned int depth,
                                       unsigned int max_depth);

  octomap::OcTreeNode* touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth);

  void expandNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth);

  std::optional<double> getGroundZ(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y);

  bool translateMap(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y, const double& z);

  bool createLocalMap(const std::string frame_id, const double horizontal_distance, const double vertical_distance, std::shared_ptr<OcTree_t>& octree);

  virtual void insertPointCloud(const geometry_msgs::Vector3& sensorOrigin, const PCLPointCloud::ConstPtr& cloud, const PCLPointCloud::ConstPtr& free_cloud,
                                double free_ray_distance, bool unknown_clear_occupied = false);

  void initialize3DLidarLUT(xyz_lut_t& lut, const SensorParams3DLidar_t sensor_params);

  void timeoutGeneric(const std::string& topic, const ros::Time& last_msg, [[maybe_unused]] const int n_pubs);

  void publishProjected2DMap(const ros::Time& rostime);
  void handlePostNodeTraversal(const ros::Time& rostime);
  void handlePreNodeTraversal(const ros::Time& rostime);

  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

  int n_sensors_3d_lidar_;


  std::vector<xyz_lut_t> sensor_3d_lidar_xyz_lut_;



  std::vector<SensorParams3DLidar_t> sensor_params_3d_lidar_;


  std::mutex mutex_lut_;

  std::vector<bool> vec_camera_info_processed_;

  // sensor model
  double _probHit_;
  double _probMiss_;
  double _thresMin_;
  double _thresMax_;


  std_msgs::ColorRGBA octree_global_color;
  nav_msgs::OccupancyGrid octree_global_gridmap;



bool octree_global_useHeightMap;
bool octree_global_useColoredMap;
bool octree_global_compressMap;
bool octree_global_incrementalUpdate;
bool octree_global_projectCompleteMap;
bool octree_global_publishFreeSpace;


double octree_global_pointcloudMinX;
double octree_global_pointcloudMaxX;
double octree_global_pointcloudMinY;
double octree_global_pointcloudMaxY;
double octree_global_pointcloudMinZ;
double octree_global_pointcloudMaxZ;
double octree_global_occupancyMinZ;
double octree_global_occupancyMaxZ;
double octree_global_minSizeX;
double octree_global_minSizeY;
double octree_global_colorFactor;
double octree_global_groundFilterDistance;
double octree_global_groundFilterAngle;
double octree_global_groundFilterPlaneDistance;
double octree_global_maxRange;
double octree_global_minRange;
double octree_global_res;

unsigned octree_global_treeDepth;
unsigned octree_global_maxTreeDepth;
unsigned octree_global_multires2DScale;

octomap::OcTreeKey octree_global_updateBBXMin;
octomap::OcTreeKey octree_global_updateBBXMax;
octomap::OcTreeKey octree_global_paddedMinKey;

void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const;
void update2DMap(const OcTree_t::iterator& it, bool occupied);


  inline unsigned mapIdx(int i, int j) const {
    return octree_global_gridmap.info.width * j + i;
  }


inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
return mapIdx((key[0] - octree_global_paddedMinKey[0]) / octree_global_multires2DScale,
                (key[1] - octree_global_paddedMinKey[1]) / octree_global_multires2DScale);

}

inline bool mapChanged(const nav_msgs::MapMetaData& oldMapInfo, const nav_msgs::MapMetaData& newMapInfo) {
return (    oldMapInfo.height != newMapInfo.height
            || oldMapInfo.width != newMapInfo.width
            || oldMapInfo.origin.position.x != newMapInfo.origin.position.x
            || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
}


void handleOccupiedNode(const OcTree_t::iterator& it);

void handleFreeNode(const OcTree_t::iterator& it);

void handleOccupiedNodeInBBX(const OcTree_t::iterator& it);

void handleFreeNodeInBBX(const OcTree_t::iterator& it);

void publishAll(const ros::Time& rostime);

/// hook that is called when traversing all nodes of the updated Octree (does nothing here)
virtual void handleNode(const OcTree_t::iterator& it) {};

/// hook that is called when traversing all nodes of the updated Octree in the updated area (does nothing here)
virtual void handleNodeInBBX(const OcTree_t::iterator& it) {};

  /// Test if key is within update area of map (2D, ignores height)
  inline bool isInUpdateBBX(const OcTree_t::iterator& it) const {
    // 2^(tree_depth-depth) voxels wide:
    unsigned voxelWidth = (1 << (octree_global_maxTreeDepth - it.getDepth()));
    octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
    return (key[0] + voxelWidth >= octree_global_updateBBXMin[0]
            && key[1] + voxelWidth >= octree_global_updateBBXMin[1]
            && key[0] <= octree_global_updateBBXMax[0]
            && key[1] <= octree_global_updateBBXMax[1]);
  }


};
}