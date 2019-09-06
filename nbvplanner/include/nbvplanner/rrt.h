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

#ifndef RRTTREE_H_
#define RRTTREE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <kdtree/kdtree.h>
#include <nbvplanner/tree.h>
#include <nbvplanner/mesh_structure.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {

class RrtTree : public TreeBase<Eigen::Vector4d>
{
 public:
  typedef Eigen::Vector4d StateVec;

  RrtTree();
  RrtTree(mesh::StlMesh * mesh, volumetric_mapping::OctomapManager * manager);
  ~RrtTree();
  virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose);
  virtual void setStateFromOdometryMsg(const nav_msgs::Odometry& pose);
  virtual void setPeerStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose, int n_peer);
  virtual void initialize();
  virtual void iterate(int iterations);
  virtual std::vector<geometry_msgs::Pose> getBestEdge(std::string targetFrame);
  virtual void clear();
  virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(std::string targetFrame);
  virtual void memorizeBestBranch();
  void publishNode(Node<StateVec> * node);
  /**
   * @param[out] gain_nodes nodes that contributed to the (original) gain
   * @param[in] predicted_octomap_manager octomap manager for the predicted map
   * @param[out] predictive_gain_nodes nodes that contributed to the gain
   *                from predicted map
   * @param[out] predictive_gain gain from the predicted map
   */
  double gain(
      StateVec state,
      std::vector<Eigen::Vector3d> *gain_nodes=nullptr,
      const volumetric_mapping::OctomapManager * const
                predicted_octomap_manager=nullptr,
      std::vector<Eigen::Vector3d> *predictive_gain_nodes=nullptr,
      double *predictive_gain=nullptr) const;

  std::vector<geometry_msgs::Pose> samplePath(StateVec start, StateVec end,
                                              std::string targetFrame);

  std::vector<StateVec> getBestBranch() { return bestBranchMemory_; }

  void setBestBranch(const std::vector<StateVec> &bestBranch) {
    bestBranchMemory_ = bestBranch;
  }

 protected:
  kdtree * kdTree_;
  std::stack<StateVec> history_;
  std::vector<StateVec> bestBranchMemory_;
  int g_ID_;
  int iterationCount_;
  std::fstream fileTree_;
  std::fstream filePath_;
  std::fstream fileResponse_;
  std::string logFilePath_;
  std::vector<double> inspectionThrottleTime_;
};
}

#endif
