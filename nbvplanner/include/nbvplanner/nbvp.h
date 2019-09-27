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
#include <memory>
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

#include <scene_completion_3d_interface/scene_completion_3d_interface.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {

enum BestPathState
{
  NOT_FOUND=0, OKAY, ALMOST_OKAY, LEN
};

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
  /** Whether to use Information Gain from predicted map */
  bool use_predictive_ig_;
  /**
   * Whether to compute both predictive and original trajectories
   *    Only valid when use_predictive_ig_ is true
   */
  bool compute_both_ig_trajectories_;

  using SceneCompletion3dInterface =
      scene_completion_3d_interface::SceneCompletion3dInterface;
  std::unique_ptr<SceneCompletion3dInterface> scene_completion_interface_;

 public:
  typedef std::vector<stateVec> vector_t;

  /** (RRT) Tree using predictive/original/both information gain */
  std::shared_ptr< TreeBase<stateVec> > rrt_tree_;

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

  /** Initialize the (RRT) tree(s) */
  void initializeTrees();

  /**
   * Get best path from a given tree
   * Optionally return immediate gain and gain_nodes (voxels)
   * @return state of the query
   */
  BestPathState getBestPath(
      const std::shared_ptr< TreeBase<stateVec> > &tree,
      const std::string &frame_id,
      std::vector<geometry_msgs::Pose> &path,
      float * const gain=nullptr,
      std::vector<Eigen::Vector3d> * const gain_nodes=nullptr);

  /** Get complete scene from the map created thus far */
  bool getCompletedOcTree(
      scene_completion_3d_interface::OcTreeTPtr &completed_octree) const;

  bool getCompletedOcTreeManager(
      std::shared_ptr<volumetric_mapping::OctomapManager>
          &completed_octree_manger) const;

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
