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

#ifndef TREE_H_
#define TREE_H_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_world/octomap_manager.h>
#include <multiagent_collision_check/Segment.h>
#include <nbvplanner/mesh_structure.h>

namespace nbvInspection {

struct Params
{
  std::vector<double> camPitch_;
  std::vector<double> camHorizontal_;
  std::vector<double> camVertical_;
  std::vector<std::vector<Eigen::Vector3d> > camBoundNormals_;

  double igProbabilistic_;
  double igFree_;
  double igOccupied_;
  double igUnmapped_;
  double igArea_;
  double gainRange_;
  double degressiveCoeff_;
  double zero_gain_;

  double v_max_;
  double dyaw_max_;
  double dOvershoot_;
  double extensionRange_;
  bool exact_root_;
  int initIterations_;
  int cuttoffIterations_;
  double dt_;

  double minX_;
  double minY_;
  double minZ_;
  double maxX_;
  double maxY_;
  double maxZ_;
  bool softBounds_;
  Eigen::Vector3d boundingBox_;

  double meshResolution_;

  ros::Publisher inspectionPath_;
  std::string navigationFrame_;

  bool log_;
  double log_throttle_;
  double pcl_throttle_;
  double inspection_throttle_;
};

enum InfoGainType {
  ORIGINAL = 0, PREDICTIVE, IG_TYPE_LEN
};

template<typename stateVec>
class Node
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Node();
  ~Node();
  stateVec state_;
  Node * parent_;
  std::vector<Node*> children_;

  double original_gain_;
  double predictive_gain_;
  std::vector<Eigen::Vector3d> original_gain_nodes_;
  std::vector<Eigen::Vector3d> predictive_gain_nodes_;

  double distance_;
};

template<typename stateVec>
class TreeBase
{
 protected:
  Params params_;
  int counter_;
  double bestOriginalGain_;
  double bestPredictiveGain_;

  Node<stateVec> * bestOriginalNode_;
  Node<stateVec> * bestPredictiveNode_;
  Node<stateVec> * rootNode_;
  mesh::StlMesh * mesh_;
  volumetric_mapping::OctomapManager * manager_;
  stateVec root_;
  stateVec exact_root_;
  std::vector<std::vector<Eigen::Vector3d>*> segments_;
  std::vector<std::string> agentNames_;

  std::shared_ptr<volumetric_mapping::OctomapManager>
      predictedOctomapManager_;
 public:
  TreeBase();
  TreeBase(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
  ~TreeBase();
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose) = 0;
  virtual void setStateFromOdometryMsg(const nav_msgs::Odometry& pose) = 0;
  virtual void setPeerStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose, int n_peer) = 0;
  void setPeerStateFromPoseMsg1(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void setPeerStateFromPoseMsg2(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void setPeerStateFromPoseMsg3(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void evade(const multiagent_collision_check::Segment& segmentMsg);
  virtual void iterate(int iterations) = 0;
  virtual void initialize() = 0;
  virtual std::vector<geometry_msgs::Pose> getBestEdge(
      const InfoGainType gainType,
      std::string targetFrame,
      double * const gain=nullptr,
      std::vector<Eigen::Vector3d> * const gainNodes=nullptr) = 0;
  virtual std::vector<geometry_msgs::Pose> getBestFullTrajectory(
      const InfoGainType gainType,
      std::string targetFrame,
      double * const gain=nullptr,
      std::vector<Eigen::Vector3d> * const gainNodes=nullptr) = 0;
  virtual void clear() = 0;
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(
      std::string targetFrame,
      double * const gain=nullptr,
      std::vector<Eigen::Vector3d> * const gain_nodes=nullptr) = 0;

  virtual void memorizeBestBranch() = 0;
  virtual std::vector<stateVec> getBestBranch() = 0;
  virtual void setBestBranch(const std::vector<stateVec> &bestBranch) = 0;

  void setParams(Params params);
  int getCounter();
  bool originalGainFound();
  bool predictiveGainFound();
  void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);

  void setPredictedOctomapManager(
      std::shared_ptr<volumetric_mapping::OctomapManager> &predictedManager)
  {
    predictedOctomapManager_ = predictedManager;
  }
};
}

#endif
