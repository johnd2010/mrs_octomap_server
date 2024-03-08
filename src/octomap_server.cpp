/* includes //{ */


#include <mrs_octomap_server/octomap_server.h>
// #include <mrs_octomap_server/OctomapServer.h>

//}

namespace mrs_octomap_server
{

void OctomapServer::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
  

  ros::Time::waitForValid();

  /* params //{ */

  mrs_lib::ParamLoader paramloader(nh_, ros::this_node::getName());
  //loading params of mrs_octomap
  {
  
  paramloader.loadParam("simulation", _simulation_);
  paramloader.loadParam("uav_name", _uav_name_);

  paramloader.loadParam("scope_timer/enabled", _scope_timer_enabled_);

  paramloader.loadParam("map_while_grounded", _map_while_grounded_);

  paramloader.loadParam("persistency/enabled", _persistency_enabled_);
  paramloader.loadParam("persistency/save_time", _persistency_save_time_);
  paramloader.loadParam("persistency/map_name", _persistency_map_name_);
  paramloader.loadParam("persistency/align_altitude/enabled", _persistency_align_altitude_enabled_);
  paramloader.loadParam("persistency/align_altitude/ground_detection_distance", _persistency_align_altitude_distance_);
  paramloader.loadParam("persistency/align_altitude/robot_height", _robot_height_);

  paramloader.loadParam("global_map/publisher_rate", _global_map_publisher_rate_);
  paramloader.loadParam("global_map/creation_rate", _global_map_creator_rate_);
  paramloader.loadParam("global_map/enabled", _global_map_enabled_);
  paramloader.loadParam("global_map/compress", _global_map_compress_);
  paramloader.loadParam("global_map/publish_full", _global_map_publish_full_);
  paramloader.loadParam("global_map/publish_binary", _global_map_publish_binary_);

  paramloader.loadParam("local_map/size/max_width", _local_map_width_max_);
  paramloader.loadParam("local_map/size/max_height", _local_map_height_max_);
  paramloader.loadParam("local_map/size/min_width", _local_map_width_min_);
  paramloader.loadParam("local_map/size/min_height", _local_map_height_min_);
  paramloader.loadParam("local_map/size/duty_high_threshold", _local_map_duty_high_threshold_);
  paramloader.loadParam("local_map/size/duty_low_threshold", _local_map_duty_low_threshold_);
  paramloader.loadParam("local_map/publisher_rate", _local_map_publisher_rate_);
  paramloader.loadParam("local_map/publish_full", _local_map_publish_full_);
  paramloader.loadParam("local_map/publish_binary", _local_map_publish_binary_);

  local_map_width_  = _local_map_width_max_;
  local_map_height_ = _local_map_height_max_;

  paramloader.loadParam("resolution", octree_resolution_);
  paramloader.loadParam("world_frame_id", _world_frame_);
  paramloader.loadParam("robot_frame_id", _robot_frame_);

  paramloader.loadParam("map_path", _map_path_);

  paramloader.loadParam("unknown_rays/update_free_space", _unknown_rays_update_free_space_);
  paramloader.loadParam("unknown_rays/clear_occupied", _unknown_rays_clear_occupied_);
  paramloader.loadParam("unknown_rays/ray_distance", _unknown_rays_distance_);

  paramloader.loadParam("sensor_params/3d_lidar/n_sensors", n_sensors_3d_lidar_);

  for (int i = 0; i < n_sensors_3d_lidar_; i++) {

    std::stringstream max_range_param_name;
    max_range_param_name << "sensor_params/3d_lidar/sensor_" << i << "/max_range";

    std::stringstream free_ray_distance_param_name;
    free_ray_distance_param_name << "sensor_params/3d_lidar/sensor_" << i << "/free_ray_distance";

    std::stringstream horizontal_rays_param_name;
    horizontal_rays_param_name << "sensor_params/3d_lidar/sensor_" << i << "/horizontal_rays";

    std::stringstream vertical_rays_param_name;
    vertical_rays_param_name << "sensor_params/3d_lidar/sensor_" << i << "/vertical_rays";

    std::stringstream vfov_param_name;
    vfov_param_name << "sensor_params/3d_lidar/sensor_" << i << "/vertical_fov_angle";

    std::stringstream update_free_space_param_name;
    update_free_space_param_name << "sensor_params/3d_lidar/sensor_" << i << "/unknown_rays/update_free_space";

    std::stringstream clear_occupied_param_name;
    clear_occupied_param_name << "sensor_params/3d_lidar/sensor_" << i << "/unknown_rays/clear_occupied";

    std::stringstream free_ray_distance_unknown_param_name;
    free_ray_distance_unknown_param_name << "sensor_params/3d_lidar/sensor_" << i << "/unknown_rays/free_ray_distance_unknown";

    SensorParams3DLidar_t params;

    paramloader.loadParam(max_range_param_name.str(), params.max_range);
    paramloader.loadParam(free_ray_distance_param_name.str(), params.free_ray_distance);
    paramloader.loadParam(horizontal_rays_param_name.str(), params.horizontal_rays);
    paramloader.loadParam(vertical_rays_param_name.str(), params.vertical_rays);
    paramloader.loadParam(vfov_param_name.str(), params.vertical_fov);
    paramloader.loadParam(update_free_space_param_name.str(), params.update_free_space);
    paramloader.loadParam(clear_occupied_param_name.str(), params.clear_occupied);
    paramloader.loadParam(free_ray_distance_unknown_param_name.str(), params.free_ray_distance_unknown);

    sensor_params_3d_lidar_.push_back(params);
  }

  paramloader.loadParam("sensor_model/hit", _probHit_);
  paramloader.loadParam("sensor_model/miss", _probMiss_);
  paramloader.loadParam("sensor_model/min", _thresMin_);
  paramloader.loadParam("sensor_model/max", _thresMax_);

  if (!paramloader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", ros::this_node::getName().c_str());
    ros::requestShutdown();
  }
}


{
  double probHit, probMiss, thresMin, thresMax;

  paramloader.loadParam("height_map", octree_global_useHeightMap, true);
  paramloader.loadParam("colored_map", octree_global_useColoredMap, false);
  paramloader.loadParam("color_factor", octree_global_colorFactor, 0.8);

  paramloader.loadParam("pointcloud_min_x", octree_global_pointcloudMinX,-std::numeric_limits<double>::max());
  paramloader.loadParam("pointcloud_max_x", octree_global_pointcloudMaxX,std::numeric_limits<double>::max());
  paramloader.loadParam("pointcloud_min_y", octree_global_pointcloudMinY,-std::numeric_limits<double>::max());
  paramloader.loadParam("pointcloud_max_y", octree_global_pointcloudMaxY,std::numeric_limits<double>::max());
  paramloader.loadParam("pointcloud_min_z", octree_global_pointcloudMinZ,-std::numeric_limits<double>::max());
  paramloader.loadParam("pointcloud_max_z", octree_global_pointcloudMaxZ,std::numeric_limits<double>::max());
  paramloader.loadParam("occupancy_min_z", octree_global_occupancyMinZ,-std::numeric_limits<double>::max());
  paramloader.loadParam("occupancy_max_z", octree_global_occupancyMaxZ,std::numeric_limits<double>::max());
  paramloader.loadParam("min_x_size", octree_global_minSizeX,0.0);
  paramloader.loadParam("min_y_size", octree_global_minSizeY,0.0);

  // distance of points from plane for RANSAC
  paramloader.loadParam("ground_filter/distance", octree_global_groundFilterDistance, octree_global_groundFilterDistance);
  // angular derivation of found plane:
  paramloader.loadParam("ground_filter/angle", octree_global_groundFilterAngle, octree_global_groundFilterAngle);
  // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
  paramloader.loadParam("ground_filter/plane_distance", octree_global_groundFilterPlaneDistance, octree_global_groundFilterPlaneDistance);

  paramloader.loadParam("sensor_model/max_range", octree_global_maxRange, -1.0);
  paramloader.loadParam("sensor_model/min_range", octree_global_minRange, -1.0);

  paramloader.loadParam("resolution", octree_global_res, 0.05);
  paramloader.loadParam("sensor_model/hit", probHit, 0.7);
  paramloader.loadParam("sensor_model/miss", probMiss, 0.4);
  paramloader.loadParam("sensor_model/min", thresMin, 0.12);
  paramloader.loadParam("sensor_model/max", thresMax, 0.97);
  paramloader.loadParam("compress_map", octree_global_compressMap, octree_global_compressMap);
  paramloader.loadParam("incremental_2D_projection", octree_global_incrementalUpdate, octree_global_incrementalUpdate);

  if ( (octree_global_pointcloudMinZ > 0.0 || octree_global_pointcloudMaxZ < 0.0)){
    ROS_WARN_STREAM("You enabled ground filtering but incoming pointclouds will be pre-filtered in ["
              <<octree_global_pointcloudMinZ <<", "<< octree_global_pointcloudMaxZ << "], excluding the ground level z=0. "
              << "This will not work.");
  }

  if (octree_global_useHeightMap && octree_global_useColoredMap) {
    ROS_WARN_STREAM("You enabled both height map and RGB color registration. This is contradictory. Defaulting to height map.");
    octree_global_useColoredMap = false;
  }

  if (octree_global_useColoredMap) {
#ifdef COLOR_OCTOMAP_SERVER
    ROS_INFO_STREAM("Using RGB color registration (if information available)");
#else
    ROS_ERROR_STREAM("Colored map requested in launch file - node not running/compiled to support colors, please define COLOR_OCTOMAP_SERVER and recompile or launch the octomap_color_server node");
#endif
  }



  double r, g, b, a;
  paramloader.loadParam("color/r", r, 0.0);
  paramloader.loadParam("color/g", g, 0.0);
  paramloader.loadParam("color/b", b, 1.0);
  paramloader.loadParam("color/a", a, 1.0);
  octree_global_color.r = r;
  octree_global_color.g = g;
  octree_global_color.b = b;
  octree_global_color.a = a;




//JOHN CHECK THESE TWO 
  // m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, _world_frame_, 5);
  // m_tfPointCloudSub->registerCallback(boost::bind(&OctomapServer::insertCloudCallback, this, boost::placeholders::_1));
//JOHN CHECK THESE TWO 

}


  //}

  /* initialize sensor LUT model //{ */

  for (int i = 0; i < n_sensors_3d_lidar_; i++) {

    xyz_lut_t lut_table;

    sensor_3d_lidar_xyz_lut_.push_back(lut_table);

    initialize3DLidarLUT(sensor_3d_lidar_xyz_lut_[i], sensor_params_3d_lidar_[i]);
  }

  //}

  /* initialize octomap object & params //{ */

    // initialize octomap object & params
  
  
  

  octree_global_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_global_->setProbHit(_probHit_);
  octree_global_->setProbMiss(_probMiss_);
  octree_global_->setClampingThresMin(_thresMin_);
  octree_global_->setClampingThresMax(_thresMax_);
  octree_global_treeDepth = octree_global_->getTreeDepth();
  octree_global_maxTreeDepth = octree_global_treeDepth;
  octree_global_gridmap.info.resolution = octree_global_res;

  octree_local_0_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_local_0_->setProbHit(_probHit_);
  octree_local_0_->setProbMiss(_probMiss_);
  octree_local_0_->setClampingThresMin(_thresMin_);
  octree_local_0_->setClampingThresMax(_thresMax_);

  octree_local_1_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_local_1_->setProbHit(_probHit_);
  octree_local_1_->setProbMiss(_probMiss_);
  octree_local_1_->setClampingThresMin(_thresMin_);
  octree_local_1_->setClampingThresMax(_thresMax_);

  octree_local_ = octree_local_0_;


  if (_persistency_enabled_ && _persistency_align_altitude_enabled_) {
    octrees_initialized_ = false;
  } else {
    octrees_initialized_ = true;
  }

  //}

  /* transformer //{ */

  transformer_ = std::make_unique<mrs_lib::Transformer>("OctomapServer");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->setLookupTimeout(ros::Duration(0.5));
  transformer_->retryLookupNewest(false);

  //}

  /* publishers //{ */

  pub_map_global_full_   = nh_.advertise<octomap_msgs::Octomap>("octomap_global_full_out", 1);
  pub_map_global_binary_ = nh_.advertise<octomap_msgs::Octomap>("octomap_global_binary_out", 1);
  global_mapPub           = nh_.advertise<nav_msgs::OccupancyGrid>("projected_map_mrs", 5);
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
  sh_height_               = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "height_in");
  sh_clear_box_            = mrs_lib::SubscribeHandler<mrs_octomap_server::PoseWithSize>(shopts, "clear_box_in");

  for (int i = 0; i < n_sensors_3d_lidar_; i++) {

    std::stringstream ss;
    ss << "lidar_3d_" << i << "_in";

    sh_3dlaser_pc2_.push_back(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(
        shopts, ss.str(), ros::Duration(2.0), std::bind(&OctomapServer::callback3dLidarCloud2, this, std::placeholders::_1, LIDAR_3D, i, ss.str(), false)));

    std::stringstream ss2;
    ss2 << "lidar_3d_" << i << "_over_max_range_in";

    sh_3dlaser_pc2_.push_back(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(
        shopts, ss2.str(), ros::Duration(2.0), std::bind(&OctomapServer::callback3dLidarCloud2, this, std::placeholders::_1, LIDAR_3D, i, ss.str(), true)));
  }


  //}

  /* service servers //{ */


  //}

  /* timers //{ */

  if (_global_map_enabled_) {
    timer_global_map_publisher_ = nh_.createTimer(ros::Rate(_global_map_publisher_rate_), &OctomapServer::timerGlobalMapPublisher, this);
    timer_global_map_creator_   = nh_.createTimer(ros::Rate(_global_map_creator_rate_), &OctomapServer::timerGlobalMapCreator, this);
  }

  timer_local_map_publisher_ = nh_.createTimer(ros::Rate(_local_map_publisher_rate_), &OctomapServer::timerLocalMapPublisher, this);

  timer_local_map_resizer_ = nh_.createTimer(ros::Rate(1.0), &OctomapServer::timerLocalMapResizer, this);

  if (_persistency_enabled_) {
    timer_persistency_ = nh_.createTimer(ros::Rate(1.0 / _persistency_save_time_), &OctomapServer::timerPersistency, this);
  }

  if (_persistency_enabled_ && _persistency_align_altitude_enabled_) {
    timer_altitude_alignment_ = nh_.createTimer(ros::Rate(1.0), &OctomapServer::timerAltitudeAlignment, this);
  }

  //}

  /* scope timer logger //{ */

  const std::string scope_timer_log_filename = paramloader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(scope_timer_log_filename, scope_timer_enabled_);

  //}

  is_initialized_ = true;

  ROS_INFO("[%s]: Initialized", ros::this_node::getName().c_str());
}

//}

// | --------------------- topic callbacks -------------------- |



//}

/* callback3dLidarCloud2() //{ */

void OctomapServer::callback3dLidarCloud2(const sensor_msgs::PointCloud2::ConstPtr msg, const SensorType_t sensor_type, const int sensor_id,
                                          const std::string topic, const bool pcl_over_max_range) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }


  if (!_map_while_grounded_) {

    if (!sh_control_manager_diag_.hasMsg()) {

      ROS_WARN_THROTTLE(1.0, "[OctomapServer]: missing control manager diagnostics, can not integrate data!");
      return;

    } else {

      ros::Time last_time = sh_control_manager_diag_.lastMsgTime();

      if ((ros::Time::now() - last_time).toSec() > 1.0) {
        ROS_WARN_THROTTLE(1.0, "[OctomapServer]: control manager diagnostics too old, can not integrate data!");
        return;
      }

      // TODO is this the best option?
      if (!sh_control_manager_diag_.getMsg()->flying_normally) {
        ROS_INFO_THROTTLE(1.0, "[OctomapServer]: not flying normally, therefore, not integrating data");
        return;
      }
    }
  }

  sensor_msgs::PointCloud2ConstPtr cloud = msg;

  ros::Time time_start = ros::Time::now();

  PCLPointCloud::Ptr pc              = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr hit_pc          = boost::make_shared<PCLPointCloud>();

  pcl::fromROSMsg(*cloud, *pc);

  auto res = transformer_->getTransform(cloud->header.frame_id, _world_frame_, cloud->header.stamp);

  if (!res) {
    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: callback3dLidarCloud2(): could not find tf from %s to %s", cloud->header.frame_id.c_str(), _world_frame_.c_str());
    return;
  }

  Eigen::Matrix4f                 sensorToWorld;
  geometry_msgs::TransformStamped sensorToWorldTf = res.value();
  pcl_ros::transformAsMatrix(sensorToWorldTf.transform, sensorToWorld);

  double max_range;

  if (!pcl_over_max_range) {

    // generate sensor lookup table for free space raycasting based on pointcloud dimensions
    if (cloud->height == 1 || cloud->width == 1) {
      ROS_WARN_THROTTLE(2.0, "Incoming pointcloud from %s #%d on topic %s is unorganized! Free space raycasting of unknows rays won't work properly!",
                        _sensor_names_[sensor_type].c_str(), sensor_id, topic.c_str());
    }

    switch (sensor_type) {

      case LIDAR_3D: {

        std::scoped_lock lock(mutex_lut_);

        // change number of rays if it differs from the pointcloud dimensions
        if (sensor_params_3d_lidar_[sensor_id].horizontal_rays != cloud->width || sensor_params_3d_lidar_[sensor_id].vertical_rays != cloud->height) {
          sensor_params_3d_lidar_[sensor_id].horizontal_rays = cloud->width;
          sensor_params_3d_lidar_[sensor_id].vertical_rays   = cloud->height;
          ROS_INFO("[OctomapServer]: Changing sensor params for lidar %d to %d horizontal rays, %d vertical rays.", sensor_id,
                   sensor_params_3d_lidar_[sensor_id].horizontal_rays, sensor_params_3d_lidar_[sensor_id].vertical_rays);
          initialize3DLidarLUT(sensor_3d_lidar_xyz_lut_[sensor_id], sensor_params_3d_lidar_[sensor_id]);
        }

        max_range = sensor_params_3d_lidar_[sensor_id].max_range;

        break;
      }
      default: {

        break;
      }
    }
  }

  // get raycasting parameters
  double free_ray_distance      = 0;
  bool   unknown_clear_occupied = false;
  switch (sensor_type) {
    case LIDAR_3D: {
      std::scoped_lock lock(mutex_lut_);
      free_ray_distance      = sensor_params_3d_lidar_[sensor_id].free_ray_distance;
      unknown_clear_occupied = sensor_params_3d_lidar_[sensor_id].clear_occupied;
      break;
    }
    default: {
      break;
    }
  }

  // points that are over the max range from previous pcl filtering, update only free space
  if (pcl_over_max_range) {

    free_vectors_pc->swap(*pc);

  } else {

    // go through the pointcloud
    for (int i = 0; i < pc->size(); i++) {

      pcl::PointXYZ pt = pc->at(i);

      if ((!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))) {
        // datapoint is missing, update only free space, if desired
        vec3_t ray_vec;
        double raycasting_distance       = 0;
        bool   unknown_update_free_space = false;
        switch (sensor_type) {
          case LIDAR_3D: {
            std::scoped_lock lock(mutex_lut_);
            ray_vec                   = sensor_3d_lidar_xyz_lut_[sensor_id].directions.col(i);
            raycasting_distance       = sensor_params_3d_lidar_[sensor_id].free_ray_distance_unknown;
            unknown_update_free_space = sensor_params_3d_lidar_[sensor_id].update_free_space;
            break;
          }
          default: {
            break;
          }
        }

        if (unknown_update_free_space) {
          pcl::PointXYZ temp_pt;

          temp_pt.x = ray_vec(0) * float(raycasting_distance);
          temp_pt.y = ray_vec(1) * float(raycasting_distance);
          temp_pt.z = ray_vec(2) * float(raycasting_distance);

          free_vectors_pc->push_back(temp_pt);
        }
      } else if ((pow(pt.x, 2) + pow(pt.y, 2) + pow(pt.z, 2)) > pow(max_range, 2)) {
        // point is over the max range, update only free space
        free_vectors_pc->push_back(pt);
      } else {
        // point is ok
        hit_pc->push_back(pt);
      }

    }
  }

  free_vectors_pc->header = pc->header;

  // transform to the map frame

  pcl::transformPointCloud(*hit_pc, *hit_pc, sensorToWorld);
  pcl::transformPointCloud(*free_vectors_pc, *free_vectors_pc, sensorToWorld);

  hit_pc->header.frame_id          = _world_frame_;
  free_vectors_pc->header.frame_id = _world_frame_;

  insertPointCloud(sensorToWorldTf.transform.translation, hit_pc, free_vectors_pc, free_ray_distance, unknown_clear_occupied);

  const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation);

  {
    std::scoped_lock lock(mutex_avg_time_cloud_insertion_);

    ros::Time time_end = ros::Time::now();

    double exec_duration = (time_end - time_start).toSec();

    double coef               = 0.5;
    avg_time_cloud_insertion_ = coef * avg_time_cloud_insertion_ + (1.0 - coef) * exec_duration;

    ROS_INFO_THROTTLE(1.0, "[OctomapServer]: avg cloud insertion time = %.3f sec", avg_time_cloud_insertion_);
  }
}  // namespace mrs_octomap_server



// | ------------------------- timers ------------------------- |

/* timerGlobalMapPublisher() //{ */

void OctomapServer::timerGlobalMapPublisher([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: full map publisher timer spinning");

  size_t octomap_size;

  {
    std::scoped_lock lock(mutex_octree_global_);

    octomap_size = octree_global_->size();
  }

  if (octomap_size <= 1) {
    ROS_WARN("[%s]: Nothing to publish, octree is empty", ros::this_node::getName().c_str());
    return;
  }

  /* if (_global_map_compress_) { */
  /*   octree_global_->prune(); */
  /* } */

  if (pub_map_global_full_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    bool success = false;

    {
      std::scoped_lock lock(mutex_octree_global_);

      mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::globalMapFullPublish", scope_timer_logger_, _scope_timer_enabled_);

      success = octomap_msgs::fullMapToMsg(*octree_global_, map);
    }

    if (success) {
      pub_map_global_full_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing global octomap to full representation");
    }
  }

  if (_global_map_publish_binary_) {

    octomap_msgs::Octomap map;
    
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    bool success = false;
    

    {
      std::scoped_lock lock(mutex_octree_global_);

      mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::globalMapBinaryPublish", scope_timer_logger_, _scope_timer_enabled_);

      success = octomap_msgs::binaryMapToMsg(*octree_global_, map);
    }

    if (success) {
      pub_map_global_binary_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing global octomap to binary representation");
    }
  }
  publishAll(ros::Time::now());
}

//}

/* timerGlobalMapCreator() //{ */

void OctomapServer::timerGlobalMapCreator([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::timerGlobalMapCreator", scope_timer_logger_, _scope_timer_enabled_);

  ROS_INFO_ONCE("[OctomapServer]: global map creator timer spinning");

  // copy the local map into a buffer

  std::shared_ptr<OcTree_t> local_map_tmp_;
  {
    std::scoped_lock lock(mutex_octree_local_);

    local_map_tmp_ = std::make_shared<OcTree_t>(*octree_local_);
  }

  local_map_tmp_->expand();

  {
    std::scoped_lock lock(mutex_octree_global_);

    copyLocalMap(local_map_tmp_, octree_global_);
  }
}

//}

/* timerLocalMapPublisher() //{ */

void OctomapServer::timerLocalMapPublisher([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: local map publisher timer spinning");

  size_t octomap_size = octree_local_->size();

  if (octomap_size <= 1) {
    ROS_WARN("[%s]: Nothing to publish, octree_local_, octree is empty", ros::this_node::getName().c_str());
    return;
  }

  // if (_local_map_publish_full_) {

  //   octomap_msgs::Octomap map;
  //   map.header.frame_id = _world_frame_;
  //   map.header.stamp    = ros::Time::now();  // TODO

  //   bool success = false;

  //   {
  //     std::scoped_lock lock(mutex_octree_local_);

  //     mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::localMapFullPublish", scope_timer_logger_, _scope_timer_enabled_);

  //     success = octomap_msgs::fullMapToMsg(*octree_local_, map);
  //   }

  //   if (success) {
  //     pub_map_local_full_.publish(map);
  //   } else {
  //     ROS_ERROR("[OctomapServer]: error serializing local octomap to full representation");
  //   }
  // }

  // if (_local_map_publish_binary_) {

  //   octomap_msgs::Octomap map;
  //   map.header.frame_id = _world_frame_;
  //   map.header.stamp    = ros::Time::now();  // TODO

  //   bool success = false;

  //   {
  //     std::scoped_lock lock(mutex_octree_local_);

  //     mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::localMapBinaryPublish", scope_timer_logger_, _scope_timer_enabled_);

  //     success = octomap_msgs::binaryMapToMsg(*octree_local_, map);
  //   }

  //   if (success) {
  //     pub_map_local_binary_.publish(map);
  //   } else {
  //     ROS_ERROR("[OctomapServer]: error serializing local octomap to binary representation");
  //   }
  // }
}

//}

/* timerLocalMapResizer() //{ */

void OctomapServer::timerLocalMapResizer([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: local map resizer timer spinning");

  auto local_map_duty = mrs_lib::get_mutexed(mutex_local_map_duty_, local_map_duty_);

  {
    std::scoped_lock lock(mutex_local_map_dimensions_);

    if (local_map_duty > _local_map_duty_high_threshold_) {
      local_map_width_ -= ceil(10.0 * (local_map_duty - _local_map_duty_high_threshold_));
      local_map_height_ -= ceil(10.0 * (local_map_duty - _local_map_duty_high_threshold_));
    } else if (local_map_duty < _local_map_duty_low_threshold_) {
      local_map_width_  = local_map_width_ + 1.0f;
      local_map_height_ = local_map_height_ + 1.0f;
    }

    if (local_map_width_ < _local_map_width_min_) {
      local_map_width_ = _local_map_width_min_;
    } else if (local_map_width_ > _local_map_width_max_) {
      local_map_width_ = _local_map_width_max_;
    }

    if (local_map_height_ < _local_map_height_min_) {
      local_map_height_ = _local_map_height_min_;
    } else if (local_map_height_ > _local_map_height_max_) {
      local_map_height_ = _local_map_height_max_;
    }

    ROS_INFO("[OctomapServer]: local map - duty time: %.3f s; size: width %.3f m, height %.3f m", local_map_duty, local_map_width_, local_map_height_);

    local_map_duty = 0;
  }

  mrs_lib::set_mutexed(mutex_local_map_duty_, local_map_duty, local_map_duty_);
}

//}

/* timerPersistency() //{ */

void OctomapServer::timerPersistency([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
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

}

//}

/* timerAltitudeAlignment() //{ */

void OctomapServer::timerAltitudeAlignment([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: altitude alignment timer spinning");

  // | ---------- check for control manager diagnostics --------- |

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

  // | -------------------- check for height -------------------- |

  bool got_height = false;

  if (sh_height_.hasMsg()) {

    ros::Time last_time = sh_height_.lastMsgTime();

    if ((ros::Time::now() - last_time).toSec() < 1.0) {
      got_height = true;
    }
  }

  // | -------------------- do the alignment -------------------- |

  bool align_using_height = false;

  if (control_manager_diag->output_enabled) {

    if (!got_height) {

      ROS_INFO("[OctomapServer]: already in the air while missing height data, skipping alignment and clearing the map");

      {
        std::scoped_lock lock(mutex_octree_global_, mutex_octree_local_);

        octree_global_->clear();
        octree_local_->clear();

        octrees_initialized_ = true;
      }

      timer_altitude_alignment_.stop();

      ROS_INFO("[OctomapServer]: stopping the altitude alignment timer");

    } else {
      align_using_height = true;
    }

  } else {

    align_using_height = false;
  }

  // | ------ get the current UAV position in the map frame ----- |

  auto res = transformer_->getTransform(_robot_frame_, _world_frame_);

  double robot_x, robot_y, robot_z;

  if (res) {

    geometry_msgs::TransformStamped world_to_robot = res.value();

    robot_x = world_to_robot.transform.translation.x;
    robot_y = world_to_robot.transform.translation.y;
    robot_z = world_to_robot.transform.translation.z;

    ROS_INFO("[OctomapServer]: robot coordinates %.2f, %.2f, %.2f", robot_x, robot_y, robot_z);

  } else {

    ROS_INFO_THROTTLE(1.0, "[OctomapServer]: waiting for the tf from %s to %s", _world_frame_.c_str(), _robot_frame_.c_str());
    return;
  }

  auto ground_z = getGroundZ(octree_global_, robot_x, robot_y);

  if (!ground_z) {

    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: could not calculate the Z of the ground below");

    {
      std::scoped_lock lock(mutex_octree_global_, mutex_octree_local_);

      octree_global_->clear();
      octree_local_->clear();

      octrees_initialized_ = true;
    }

    timer_altitude_alignment_.stop();

    ROS_INFO("[OctomapServer]: stopping the altitude alignment timer");

    return;
  }

  double ground_z_should_be = 0;

  if (align_using_height) {
    ground_z_should_be = robot_z - sh_height_.getMsg()->value;
  } else {
    ground_z_should_be = robot_z - _robot_height_ - 0.5 * octree_global_->getResolution();
  }

  double offset = ground_z_should_be - ground_z.value();

  ROS_INFO("[OctomapServer]: ground is at height %.2f m", ground_z.value());
  ROS_INFO("[OctomapServer]: ground should be at height %.2f m", ground_z_should_be);
  ROS_INFO("[OctomapServer]: shifting ground by %.2f m", offset);

  translateMap(octree_global_, 0, 0, offset);
  translateMap(octree_local_, 0, 0, offset);

  octrees_initialized_ = true;

  timer_altitude_alignment_.stop();
}

//}

// | ------------------------ routines ------------------------ |

/* insertPointCloud() //{ */

void OctomapServer::insertPointCloud(const geometry_msgs::Vector3& sensorOriginTf, const PCLPointCloud::ConstPtr& cloud,
                                     const PCLPointCloud::ConstPtr& free_vectors_cloud, double free_ray_distance, bool unknown_clear_occupied) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::timerInsertPointCloud", scope_timer_logger_, _scope_timer_enabled_);

  ros::Time time_start = ros::Time::now();

  std::scoped_lock lock(mutex_octree_local_);

  auto [local_map_width, local_map_height] = mrs_lib::get_mutexed(mutex_local_map_dimensions_, local_map_width_, local_map_height_);

  const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorOriginTf);

  const float free_space_ray_len = std::min(float(free_ray_distance), float(sqrt(2 * pow(local_map_width / 2.0, 2) + pow(local_map_height / 2.0, 2))));

  octomap::KeySet occupied_cells;
  octomap::KeySet free_cells;
  octomap::KeySet free_ends;

  // all measured points: make it free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = cloud->begin(); it != cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    const float      point_distance = float((measured_point - sensor_origin).norm());

    octomap::OcTreeKey key;
    if (octree_local_->coordToKeyChecked(measured_point, key)) {
      occupied_cells.insert(key);
    }

    // move end point to distance min(free space ray len, current distance)
    measured_point = sensor_origin + (measured_point - sensor_origin).normalize() * std::min(free_space_ray_len, point_distance);

    octomap::OcTreeKey measured_key = octree_local_->coordToKey(measured_point);

    free_ends.insert(measured_key);
  }

  // FREE VECTORS
  for (PCLPointCloud::const_iterator it = free_vectors_cloud->begin(); it != free_vectors_cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    const float      point_distance = float((measured_point - sensor_origin).norm());

    octomap::KeyRay keyRay;

    // move end point to distance min(free space ray len, current distance)
    measured_point = sensor_origin + (measured_point - sensor_origin).normalize() * std::min(free_space_ray_len, point_distance);

    // check if the ray intersects a cell in the occupied list
    if (octree_local_->computeRayKeys(sensor_origin, measured_point, keyRay)) {

      octomap::KeyRay::iterator alterantive_ray_end = keyRay.end();

      if (!unknown_clear_occupied) {

        for (octomap::KeyRay::iterator it2 = keyRay.begin(), end = keyRay.end(); it2 != end; ++it2) {

          // check if the cell is occupied in the map
          auto node = octree_local_->search(*it2);

          if (node && octree_local_->isNodeOccupied(node)) {

            if (it2 == keyRay.begin()) {
              alterantive_ray_end = keyRay.begin();  // special case
            } else {
              alterantive_ray_end = it2 - 1;
            }

            break;
          }
        }
      }

      free_cells.insert(keyRay.begin(), alterantive_ray_end);
    }
  }

  // for FREE RAY ENDS
  for (octomap::KeySet::iterator it = free_ends.begin(), end = free_ends.end(); it != end; ++it) {

    octomap::point3d coords = octree_local_->keyToCoord(*it);

    octomap::KeyRay key_ray;
    if (octree_local_->computeRayKeys(sensor_origin, coords, key_ray)) {

      octomap::KeyRay::iterator alterantive_ray_end = key_ray.end();

      for (octomap::KeyRay::iterator it2 = key_ray.begin(), end = key_ray.end(); it2 != end; ++it2) {

        if (occupied_cells.count(*it2)) {

          if (it2 == key_ray.begin()) {
            alterantive_ray_end = key_ray.begin();  // special case
          } else {
            alterantive_ray_end = it2 - 1;
          }

          break;
        }
      }

      free_cells.insert(key_ray.begin(), alterantive_ray_end);
    }
  }

  octomap::OcTreeNode* root = octree_local_->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = octree_local_->coordToKey(0, 0, 0, octree_local_->getTreeDepth());
    octree_local_->setNodeValue(key, octomap::logodds(0.0));
  }

  // FREE CELLS
  for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {

    octree_local_->updateNode(*it, octree_local_->getProbMissLog());
  }

  // OCCUPIED CELLS
  for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {

    octree_local_->updateNode(*it, octree_local_->getProbHitLog());
  }

  /* octomap::OcTreeKey robot_key = octree_local_->coordToKey(robotOriginTf.x, robotOriginTf.y, robotOriginTf.z); */
  /* octree_local_->updateNode(robot_key, false); */

  // CROP THE MAP AROUND THE ROBOT
  {

    mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::localMapCopy", scope_timer_logger_, _scope_timer_enabled_);

    auto [local_map_width, local_map_height] = mrs_lib::get_mutexed(mutex_local_map_dimensions_, local_map_width_, local_map_height_);

    float x        = sensor_origin.x();
    float y        = sensor_origin.y();
    float z        = sensor_origin.z();
    // float width_2  = octree_global_pointcloudMinX;
    float height_2 = local_map_height / float(2.0);

    octomap::point3d roi_min(x + octree_global_pointcloudMinX, y + octree_global_pointcloudMinY, z - height_2);
    octomap::point3d roi_max(x + octree_global_pointcloudMaxX, y + octree_global_pointcloudMaxY, z + height_2);

    std::shared_ptr<OcTree_t> from;

    if (octree_local_idx_ == 0) {
      from              = octree_local_0_;
      octree_local_     = octree_local_1_;
      octree_local_idx_ = 1;
    } else {
      from              = octree_local_1_;
      octree_local_     = octree_local_0_;
      octree_local_idx_ = 0;
    }

    octree_local_->clear();

    copyInsideBBX2(from, octree_local_, roi_min, roi_max);
  }


  ros::Time time_end = ros::Time::now();

  {
    std::scoped_lock lock(mutex_local_map_duty_);

    local_map_duty_ += (time_end - time_start).toSec();
  }
}

//}

/* initializeLidarLUT() //{ */

void OctomapServer::initialize3DLidarLUT(xyz_lut_t& lut, const SensorParams3DLidar_t sensor_params) {

  const int                                       rangeCount         = sensor_params.horizontal_rays;
  const int                                       verticalRangeCount = sensor_params.vertical_rays;
  std::vector<std::tuple<double, double, double>> coord_coeffs;
  const double                                    minAngle = 0.0;
  const double                                    maxAngle = 2.0 * M_PI;

  const double verticalMinAngle = -sensor_params.vertical_fov / 2.0;
  const double verticalMaxAngle = sensor_params.vertical_fov / 2.0;

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
  lut.directions.resize(3, rangeCount * verticalRangeCount);
  lut.offsets.resize(3, rangeCount * verticalRangeCount);

  for (int row = 0; row < verticalRangeCount; row++) {
    for (int col = 0; col < rangeCount; col++) {
      const auto [x_coeff, y_coeff, z_coeff] = coord_coeffs.at(col * verticalRangeCount + row);
      lut.directions.col(it)                 = vec3_t(x_coeff, y_coeff, z_coeff);
      lut.offsets.col(it)                    = vec3_t(0, 0, 0);
      it++;
    }
  }
}  



/* copyInsideBBX2() //{ */

bool OctomapServer::copyInsideBBX2(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min,
                                   const octomap::point3d& p_max) {

  octomap::OcTreeKey minKey, maxKey;

  if (!from->coordToKeyChecked(p_min, minKey) || !from->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  octomap::OcTreeNode* root = to->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(0, 0, 0, to->getTreeDepth());
    to->setNodeValue(key, octomap::logodds(0.0));
  }

  for (OcTree_t::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max), end = from->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = touchNode(to, k, it.getDepth());
    node->setValue(it->getValue());
  }

  return true;
}

//}

/* copyLocalMap() //{ */

bool OctomapServer::copyLocalMap(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to) {

  octomap::OcTreeKey minKey, maxKey;

  octomap::OcTreeNode* root = to->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(0, 0, 0, to->getTreeDepth());
    to->setNodeValue(key, octomap::logodds(0.0));
  }

  for (OcTree_t::leaf_iterator it = from->begin_leafs(), end = from->end_leafs(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = touchNode(to, k, it.getDepth());
    node->setValue(it->getValue());
  }

  return true;
}

//}

/* touchNode() //{ */

octomap::OcTreeNode* OctomapServer::touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth = 0) {

  return touchNodeRecurs(octree, octree->getRoot(), key, 0, target_depth);
}

//}

/* touchNodeRecurs() //{ */

octomap::OcTreeNode* OctomapServer::touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key,
                                                    unsigned int depth, unsigned int max_depth = 0) {

  assert(node);

  // follow down to last level
  if (depth < octree->getTreeDepth() && (max_depth == 0 || depth < max_depth)) {

    unsigned int pos = octomap::computeChildIdx(key, int(octree->getTreeDepth() - depth - 1));

    /* ROS_INFO("pos: %d", pos); */
    if (!octree->nodeChildExists(node, pos)) {

      // not a pruned node, create requested child
      octree->createNodeChild(node, pos);
    }

    return touchNodeRecurs(octree, octree->getNodeChild(node, pos), key, depth + 1, max_depth);
  }

  // at last level, update node, end of recursion
  else {
    return node;
  }
}

//}

/* expandNodeRecursive() //{ */

void OctomapServer::expandNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth) {

  if (node_depth < octree->getTreeDepth()) {

    octree->expandNode(node);

    for (int i = 0; i < 8; i++) {
      auto child = octree->getNodeChild(node, i);

      expandNodeRecursive(octree, child, node_depth + 1);
    }

  } else {
    return;
  }
}

//}

/* getGroundZ() //{ */

std::optional<double> OctomapServer::getGroundZ(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y) {

  octomap::point3d p_min(float(x - _persistency_align_altitude_distance_), float(y - _persistency_align_altitude_distance_), -10000);
  octomap::point3d p_max(float(x + _persistency_align_altitude_distance_), float(y + _persistency_align_altitude_distance_), 10000);

  for (OcTree_t::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = octree->search(k);

    expandNodeRecursive(octree, node, it.getDepth());
  }

  std::vector<octomap::point3d> occupied_points;

  for (OcTree_t::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    if (octree->isNodeOccupied(*it)) {

      occupied_points.push_back(it.getCoordinate());
    }
  }

  if (occupied_points.size() < 3) {

    ROS_ERROR("[OctomapServer]: low number of points for ground z calculation");
    return {};

  } else {

    double max_z = std::numeric_limits<double>::lowest();

    for (int i = 0; i < occupied_points.size(); i++) {
      if (occupied_points[i].z() > max_z) {
        max_z = occupied_points[i].z() - (octree_resolution_ / 2.0);
      }
    }

    /* for (int i = 0; i < occupied_points.size(); i++) { */
    /*   z += occupied_points[i].z(); */
    /* } */
    /* z /= occupied_points.size(); */

    return {max_z};
  }
}

//}

void OctomapServer::handlePostNodeTraversal(const ros::Time& rostime){
  publishProjected2DMap(rostime);
}

void OctomapServer::publishProjected2DMap(const ros::Time& rostime) {
    octree_global_gridmap.header.stamp = rostime;
    global_mapPub.publish(octree_global_gridmap);
    
}


void OctomapServer::handleOccupiedNode(const OcTree_t::iterator& it){
  if (octree_global_projectCompleteMap){
    update2DMap(it, false);
  }
}

void OctomapServer::handleFreeNode(const OcTree_t::iterator& it){
  if (octree_global_projectCompleteMap){
    update2DMap(it, false);
  }
}

void OctomapServer::handleOccupiedNodeInBBX(const OcTree_t::iterator& it){
  if (!octree_global_projectCompleteMap){
    update2DMap(it, false);
  }
}


void OctomapServer::publishAll(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = octree_global_->size();
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  // init markers for free space:
  // visualization_msgs::MarkerArray freeNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  // freeNodesVis.markers.resize(octree_global_treeDepth+1);

  // geometry_msgs::Pose pose;
  // pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // init markers:
  // visualization_msgs::MarkerArray occupiedNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  // occupiedNodesVis.markers.resize(octree_global_treeDepth+1);

  // init pointcloud:
  // pcl::PointCloud<PCLPoint> pclCloud;

  // call pre-traversal hook:
  handlePreNodeTraversal(rostime);

  // now, traverse all leafs in the tree:
  for (OcTree_t::iterator it = octree_global_->begin(octree_global_maxTreeDepth),
      end = octree_global_->end(); it != end; ++it)
  {
    bool inUpdateBBX = isInUpdateBBX(it);

    // call general hook:
    handleNode(it);
    if (inUpdateBBX)
      handleNodeInBBX(it);

    if (octree_global_->isNodeOccupied(*it)){
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > octree_global_occupancyMinZ && z - half_size < octree_global_occupancyMaxZ)
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
        int r = it->getColor().r;
        int g = it->getColor().g;
        int b = it->getColor().b;
#endif

        handleOccupiedNode(it);
        if (inUpdateBBX)
          handleOccupiedNodeInBBX(it);


      }
    } else{ // node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > octree_global_occupancyMinZ && z - half_size < octree_global_occupancyMaxZ)
      {
        handleFreeNode(it);
        if (inUpdateBBX)
          handleFreeNodeInBBX(it);
        double x = it.getX();
        double y = it.getY();
      }
    }
  }

  // call post-traversal hook:
  handlePostNodeTraversal(rostime);


  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);
}



void OctomapServer::handleFreeNodeInBBX(const OcTree_t::iterator& it){
  if (!octree_global_projectCompleteMap){
    update2DMap(it, false);
  }
}

void OctomapServer::update2DMap(const OcTree_t::iterator& it, bool occupied){
  // update 2D map (occupied always overrides):

  if (it.getDepth() == octree_global_maxTreeDepth){
    unsigned idx = mapIdx(it.getKey());
    if (occupied)
      octree_global_gridmap.data[mapIdx(it.getKey())] = 100;
    else if (octree_global_gridmap.data[idx] == -1){
      octree_global_gridmap.data[idx] = 0;
    }

  } else{
    int intSize = 1 << (octree_global_maxTreeDepth - it.getDepth());
    octomap::OcTreeKey minKey=it.getIndexKey();
    for(int dx=0; dx < intSize; dx++){
      int i = (minKey[0]+dx - octree_global_paddedMinKey[0])/octree_global_multires2DScale;
      for(int dy=0; dy < intSize; dy++){
        unsigned idx = mapIdx(i, (minKey[1]+dy - octree_global_paddedMinKey[1])/octree_global_multires2DScale);
        if (occupied)
          octree_global_gridmap.data[idx] = 100;
        else if (octree_global_gridmap.data[idx] == -1){
          octree_global_gridmap.data[idx] = 0;
        }
      }
    }
  }


}

void OctomapServer::handlePreNodeTraversal(const ros::Time& rostime){
    // init projected 2D map:
    octree_global_gridmap.header.frame_id = _world_frame_;
    octree_global_gridmap.header.stamp = rostime;
    nav_msgs::MapMetaData oldMapInfo = octree_global_gridmap.info;

    // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
    double minX, minY, minZ, maxX, maxY, maxZ;
    octree_global_->getMetricMin(minX, minY, minZ);
    octree_global_->getMetricMax(maxX, maxY, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey = octree_global_->coordToKey(minPt, octree_global_maxTreeDepth);
    octomap::OcTreeKey maxKey = octree_global_->coordToKey(maxPt, octree_global_maxTreeDepth);

    ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

    // add padding if requested (= new min/maxPts in x&y):
    double halfPaddedX = 0.5*octree_global_minSizeX;
    double halfPaddedY = 0.5*octree_global_minSizeY;
    minX = std::min(minX, -halfPaddedX);
    maxX = std::max(maxX, halfPaddedX);
    minY = std::min(minY, -halfPaddedY);
    maxY = std::max(maxY, halfPaddedY);
    minPt = octomap::point3d(minX, minY, minZ);
    maxPt = octomap::point3d(maxX, maxY, maxZ);

    octomap::OcTreeKey paddedMaxKey;
    if (!octree_global_->coordToKeyChecked(minPt, octree_global_maxTreeDepth, octree_global_paddedMinKey)){
      ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
      return;
    }
    if (!octree_global_->coordToKeyChecked(maxPt, octree_global_maxTreeDepth, paddedMaxKey)){
      ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
      return;
    }

    ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", octree_global_paddedMinKey[0], octree_global_paddedMinKey[1], octree_global_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
    assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

    octree_global_multires2DScale = 1 << (octree_global_treeDepth - octree_global_maxTreeDepth);
    octree_global_gridmap.info.width = (paddedMaxKey[0] - octree_global_paddedMinKey[0])/octree_global_multires2DScale +1;
    octree_global_gridmap.info.height = (paddedMaxKey[1] - octree_global_paddedMinKey[1])/octree_global_multires2DScale +1;

    int mapOriginX = minKey[0] - octree_global_paddedMinKey[0];
    int mapOriginY = minKey[1] - octree_global_paddedMinKey[1];
    assert(mapOriginX >= 0 && mapOriginY >= 0);

    // might not exactly be min / max of octree:
    octomap::point3d origin = octree_global_->keyToCoord(octree_global_paddedMinKey, octree_global_treeDepth);
    double gridRes = octree_global_->getNodeSize(octree_global_maxTreeDepth);
    octree_global_projectCompleteMap = (std::abs(gridRes-octree_global_gridmap.info.resolution) > 1e-6);
    octree_global_gridmap.info.resolution = gridRes;
    octree_global_gridmap.info.origin.position.x = origin.x() - gridRes*0.5;
    octree_global_gridmap.info.origin.position.y = origin.y() - gridRes*0.5;
    if (octree_global_maxTreeDepth != octree_global_treeDepth){
      octree_global_gridmap.info.origin.position.x -= octree_global_res/2.0;
      octree_global_gridmap.info.origin.position.y -= octree_global_res/2.0;
    }

    // workaround for  multires. projection not working properly for inner nodes:
    // force re-building complete map
    // if (octree_global_maxTreeDepth < octree_global_treeDepth)
    octree_global_projectCompleteMap = true;


    if(octree_global_projectCompleteMap){
      ROS_DEBUG("Rebuilding complete 2D map");
      octree_global_gridmap.data.clear();
      // init to unknown:
      octree_global_gridmap.data.resize(octree_global_gridmap.info.width * octree_global_gridmap.info.height, -1);

    } 
    // else {

    //    if (mapChanged(oldMapInfo, octree_global_gridmap.info)){
    //       ROS_DEBUG("2D grid map size changed to %dx%d", octree_global_gridmap.info.width, octree_global_gridmap.info.height);
    //       adjustMapData(octree_global_gridmap, oldMapInfo);
    //    }
    //    nav_msgs::OccupancyGrid::_data_type::iterator startIt;
    //    size_t mapUpdateBBXMinX = std::max(0, (int(octree_global_updateBBXMin[0]) - int(octree_global_paddedMinKey[0]))/int(octree_global_multires2DScale));
    //    size_t mapUpdateBBXMinY = std::max(0, (int(octree_global_updateBBXMin[1]) - int(octree_global_paddedMinKey[1]))/int(octree_global_multires2DScale));
    //    size_t mapUpdateBBXMaxX = std::min(int(octree_global_gridmap.info.width-1), (int(octree_global_updateBBXMax[0]) - int(octree_global_paddedMinKey[0]))/int(octree_global_multires2DScale));
    //    size_t mapUpdateBBXMaxY = std::min(int(octree_global_gridmap.info.height-1), (int(octree_global_updateBBXMax[1]) - int(octree_global_paddedMinKey[1]))/int(octree_global_multires2DScale));

    //    assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
    //    assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);

    //    size_t numCols = mapUpdateBBXMaxX-mapUpdateBBXMinX +1;

    //    // test for max idx:
    //    uint max_idx = octree_global_gridmap.info.width*mapUpdateBBXMaxY + mapUpdateBBXMaxX;
    //    if (max_idx  >= octree_global_gridmap.data.size())
    //      ROS_ERROR("BBX index not valid: %d (max index %zu for size %d x %d) update-BBX is: [%zu %zu]-[%zu %zu]", max_idx, octree_global_gridmap.data.size(), octree_global_gridmap.info.width, octree_global_gridmap.info.height, mapUpdateBBXMinX, mapUpdateBBXMinY, mapUpdateBBXMaxX, mapUpdateBBXMaxY);

    //    // reset proj. 2D map in bounding box:
    //    for (unsigned int j = mapUpdateBBXMinY; j <= mapUpdateBBXMaxY; ++j){
    //       std::fill_n(octree_global_gridmap.data.begin() + octree_global_gridmap.info.width*j+mapUpdateBBXMinX,
    //                   numCols, -1);
    //    }

    // }
}



/* translateMap() //{ */

bool OctomapServer::translateMap(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y, const double& z) {

  ROS_INFO("[OctomapServer]: translating map by %.2f, %.2f, %.2f", x, y, z);

  octree->expand();

  // allocate the new future octree
  std::shared_ptr<OcTree_t> octree_new = std::make_shared<OcTree_t>(octree_resolution_);
  octree_new->setProbHit(octree->getProbHit());
  octree_new->setProbMiss(octree->getProbMiss());
  octree_new->setClampingThresMin(octree->getClampingThresMin());
  octree_new->setClampingThresMax(octree->getClampingThresMax());

  for (OcTree_t::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {

    auto coords = it.getCoordinate();

    coords.x() += float(x);
    coords.y() += float(y);
    coords.z() += float(z);

    auto value = it->getValue();
    auto key   = it.getKey();

    auto new_key = octree_new->coordToKey(coords);

    octree_new->setNodeValue(new_key, value);
  }

  octree_new->prune();

  octree = octree_new;

  ROS_INFO("[OctomapServer]: map translated");

  return true;
}

//}

/* timeoutGeneric() */ /*//{*/
void OctomapServer::timeoutGeneric(const std::string& topic, const ros::Time& last_msg, [[maybe_unused]] const int n_pubs) {
  ROS_WARN_THROTTLE(1.0, "[OctomapServer]: not receiving '%s' for %.3f s", topic.c_str(), (ros::Time::now() - last_msg).toSec());
}
/*//}*/

}  // namespace mrs_octomap_server

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_octomap_server::OctomapServer, nodelet::Nodelet)
