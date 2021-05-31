/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _OCTOMAP_SERVER_H_
#define _OCTOMAP_SERVER_H_

#include <chrono>

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

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <mrs_octomap_server/conversions.h>

#include <laser_geometry/laser_geometry.h>

namespace mrs_octomap_server
{

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

  virtual bool octomapBinarySrv(octomap_msgs::GetOctomapRequest& req, octomap_msgs::GetOctomapResponse& res);
  virtual bool octomapFullSrv(octomap_msgs::GetOctomapRequest& req, octomap_msgs::GetOctomapResponse& res);

  bool clearBBXSrv(octomap_msgs::BoundingBoxQueryRequest& req, octomap_msgs::BoundingBoxQueryRequest& resp);
  bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  virtual void insertCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  virtual void insertLaserScanCallback(const sensor_msgs::LaserScanConstPtr& scan);
  virtual bool openFile(const std::string& filename);

protected:
  ros::NodeHandle nh_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> m_pointCloudSub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>>   m_laserScanSub;

  static std_msgs::ColorRGBA heightMapColor(double h);

  ros::Publisher m_occupiedPointCloudPub;
  ros::Publisher m_freePointCloudPub;
  ros::Publisher m_fmarkerPub;
  ros::Publisher m_markerPub;
  ros::Publisher m_binaryMapPub;
  ros::Publisher m_fullMapPub;
  ros::Publisher m_mapPub;

  ros::ServiceServer m_octomapBinaryService;
  ros::ServiceServer m_octomapFullService;
  ros::ServiceServer m_clearBBXService;
  ros::ServiceServer m_resetService;

  tf2_ros::Buffer                             m_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

  std::shared_ptr<OcTreeT> m_octree;

  octomap::KeyRay    m_keyRay;  // temp storage for ray casting
  octomap::OcTreeKey m_updateBBXMin;
  octomap::OcTreeKey m_updateBBXMax;

  double              m_maxRange;
  std::string         m_worldFrameId;  // the map frame
  bool                m_useHeightMap;
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

  bool   m_filterGroundPlane;
  double m_ZGroundFilterDistance;

  bool   m_localMapping;
  double m_localMapDistance;

  bool   m_clearNearby;
  double m_nearbyClearingDistance;

  bool m_updateFreeSpaceUsingMissingData;


  // downprojected 2D map:
  bool                    m_incrementalUpdate;
  nav_msgs::OccupancyGrid m_gridmap;
  bool                    m_publish2DMap;
  bool                    m_mapOriginChanged;
  octomap::OcTreeKey      m_paddedMinKey;
  unsigned                m_multires2DScale;
  bool                    m_projectCompleteMap;
  bool                    m_useColoredMap;

  bool m_isInitialized;

  laser_geometry::LaserProjection projector_;

  bool clearOutsideBBX(const octomap::point3d& p_min, const octomap::point3d& p_max);
  bool clearInsideBBX(const octomap::point3d& p_min, const octomap::point3d& p_max);

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

  void         publishBinaryOctoMap(const ros::Time& rostime) const;
  void         publishFullOctoMap(const ros::Time& rostime) const;
  virtual void publishAll(const ros::Time& rostime);

  /**
   * @brief update occupancy map with a scan
   * The scans should be in the global map frame.
   *
   * @param sensorOrigin origin of the measurements for raycasting
   * @param cloud
   * @param free_cloud
   */
  virtual void insertData(const geometry_msgs::Vector3& sensorOrigin, const PCLPointCloud::ConstPtr& cloud, const PCLPointCloud::ConstPtr& free_cloud);

  /**
   * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
   * @param key
   * @return
   */
  bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  /// hook that is called before traversing all nodes
  virtual void handlePreNodeTraversal(const ros::Time& rostime);

  /// hook that is called when traversing all nodes of the updated Octree (does nothing here)
  virtual void handleNode([[maybe_unused]] const OcTreeT::iterator& it){};

  /// hook that is called when traversing all nodes of the updated Octree in the updated area (does nothing here)
  virtual void handleNodeInBBX([[maybe_unused]] const OcTreeT::iterator& it){};

  /// hook that is called when traversing occupied nodes of the updated Octree
  virtual void handleOccupiedNode(const OcTreeT::iterator& it);

  /// hook that is called when traversing occupied nodes in the updated area (updates 2D map projection here)
  virtual void handleOccupiedNodeInBBX(const OcTreeT::iterator& it);

  /// hook that is called when traversing free nodes of the updated Octree
  virtual void handleFreeNode(const OcTreeT::iterator& it);

  /// hook that is called when traversing free nodes in the updated area (updates 2D map projection here)
  virtual void handleFreeNodeInBBX(const OcTreeT::iterator& it);

  /// hook that is called after traversing all nodes
  virtual void handlePostNodeTraversal(const ros::Time& rostime);

  /// updates the downprojected 2D map as either occupied or free
  virtual void update2DMap(const OcTreeT::iterator& it, bool occupied);

  inline unsigned mapIdx(int i, int j) const {
    return m_gridmap.info.width * j + i;
  }

  inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
    return mapIdx((key[0] - m_paddedMinKey[0]) / m_multires2DScale, (key[1] - m_paddedMinKey[1]) / m_multires2DScale);
  }

  /**
   * Adjust data of map due to a change in its info properties (origin or size,
   * resolution needs to stay fixed). map already contains the new map info,
   * but the data is stored according to oldMapInfo.
   */
  void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const;

  inline bool mapChanged(const nav_msgs::MapMetaData& oldMapInfo, const nav_msgs::MapMetaData& newMapInfo) {
    return (oldMapInfo.height != newMapInfo.height || oldMapInfo.width != newMapInfo.width || oldMapInfo.origin.position.x != newMapInfo.origin.position.x ||
            oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
  }
};

}  // namespace octomap_server

#endif  // _OCTOMAP_SERVER_HPP_
