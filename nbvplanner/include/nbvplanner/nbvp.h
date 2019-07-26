/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NBVP_H_
#define NBVP_H_

#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_world/octomap_manager.h>
#include <multiagent_collision_check/Segment.h>
#include <nbvplanner/nbvp_srv.h>
#include <nbvplanner/mesh_structure.h>
#include <nbvplanner/tree.hpp>
#include <nbvplanner/rrt.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {

template<typename stateVec>
class nbvPlanner
{

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber posClient_;
  ros::Subscriber odomClient_;
  ros::Subscriber peerPosClient1_;
  ros::Subscriber peerPosClient2_;
  ros::Subscriber peerPosClient3_;
  ros::Subscriber evadeClient_;
  ros::Publisher evadePub_;
  ros::ServiceServer plannerService_;
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber pointcloud_sub_cam_up_;
  ros::Subscriber pointcloud_sub_cam_down_;

  Params params_;
  mesh::StlMesh * mesh_;
  volumetric_mapping::OctomapManager * manager_;

  bool ready_;

 public:
  typedef std::vector<stateVec> vector_t;
  TreeBase<stateVec> * tree_;

  nbvPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~nbvPlanner();
  bool setParams();
  void posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void odomCallback(const nav_msgs::Odometry& pose);
  bool plannerCallback(nbvplanner::nbvp_srv::Request& req, nbvplanner::nbvp_srv::Response& res);
  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
  void insertPointcloudWithTfCamUp(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
  void insertPointcloudWithTfCamDown(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
  void evasionCallback(const multiagent_collision_check::Segment& segmentMsg);

  /**
   * wrapper over manager_.getMapSize() with some bells and whistles
   */
  Eigen::Vector3d getMapSize();

  // state of planner and mapper
  bool isPlannerReady() { return ready_; }
  bool isMapperReady() { return manager_; }

  // get octomap manager
  /** immutable data and immutable pointer */
  const volumetric_mapping::OctomapManager * const getOctomapManagerConstConst() const {
    return manager_;
  }

  /** immutable data */
  const volumetric_mapping::OctomapManager * getOctomapManagerConst() const {
    return manager_;
  }

  /** mutable */
  volumetric_mapping::OctomapManager * getOctomapManager() const {
    return manager_;
  }
};
}

#endif // NBVP_H_
