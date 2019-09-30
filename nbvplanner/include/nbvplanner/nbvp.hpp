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

#ifndef NBVP_HPP_
#define NBVP_HPP_

#include <fstream>
#include <eigen3/Eigen/Dense>

#include <visualization_msgs/Marker.h>

#include <nbvplanner/nbvp.h>

// Convenience macro to get the absolute yaw difference
#define ANGABS(x) (fmod(fabs(x),2.0*M_PI)<M_PI?fmod(fabs(x),2.0*M_PI):2.0*M_PI-fmod(fabs(x),2.0*M_PI))

using namespace Eigen;

template<typename stateVec>
nbvInspection::nbvPlanner<stateVec>::nbvPlanner(const ros::NodeHandle& nh,
                                                const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private)
{

  manager_ = new volumetric_mapping::OctomapManager(nh_, nh_private_);

  // Set up the topics and services
  params_.inspectionPath_ = nh_.advertise<visualization_msgs::Marker>("inspectionPath", 1000);
  evadePub_ = nh_.advertise<multiagent_collision_check::Segment>("/evasionSegment", 100);
  plannerService_ = nh_.advertiseService("nbvplanner",
                                         &nbvInspection::nbvPlanner<stateVec>::plannerCallback,
                                         this);
  posClient_ = nh_.subscribe("pose", 10, &nbvInspection::nbvPlanner<stateVec>::posCallback, this);
  odomClient_ = nh_.subscribe("odometry", 10, &nbvInspection::nbvPlanner<stateVec>::odomCallback, this);

  pointcloud_sub_ = nh_.subscribe("pointcloud_throttled", 1,
                                  &nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTf,
                                  this);
  pointcloud_sub_cam_up_ = nh_.subscribe(
      "pointcloud_throttled_up", 1,
      &nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTfCamUp, this);
  pointcloud_sub_cam_down_ = nh_.subscribe(
      "pointcloud_throttled_down", 1,
      &nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTfCamDown, this);

  if (!setParams()) {
    ROS_ERROR("Could not start the planner. Parameters missing!");
  }

  // Precompute the camera field of view boundaries. The normals of the separating hyperplanes are stored
  for (int i = 0; i < params_.camPitch_.size(); i++) {
    double pitch = M_PI * params_.camPitch_[i] / 180.0;
    double camTop = (pitch - M_PI * params_.camVertical_[i] / 360.0) + M_PI / 2.0;
    double camBottom = (pitch + M_PI * params_.camVertical_[i] / 360.0) - M_PI / 2.0;
    double side = M_PI * (params_.camHorizontal_[i]) / 360.0 - M_PI / 2.0;
    Vector3d bottom(cos(camBottom), 0.0, -sin(camBottom));
    Vector3d top(cos(camTop), 0.0, -sin(camTop));
    Vector3d right(cos(side), sin(side), 0.0);
    Vector3d left(cos(side), -sin(side), 0.0);
    AngleAxisd m = AngleAxisd(pitch, Vector3d::UnitY());
    Vector3d rightR = m * right;
    Vector3d leftR = m * left;
    rightR.normalize();
    leftR.normalize();
    std::vector<Eigen::Vector3d> camBoundNormals;
    camBoundNormals.push_back(bottom);
    // ROS_INFO("bottom: (%2.2f, %2.2f, %2.2f)", bottom[0], bottom[1], bottom[2]);
    camBoundNormals.push_back(top);
    // ROS_INFO("top: (%2.2f, %2.2f, %2.2f)", top[0], top[1], top[2]);
    camBoundNormals.push_back(rightR);
    // ROS_INFO("rightR: (%2.2f, %2.2f, %2.2f)", rightR[0], rightR[1], rightR[2]);
    camBoundNormals.push_back(leftR);
    // ROS_INFO("leftR: (%2.2f, %2.2f, %2.2f)", leftR[0], leftR[1], leftR[2]);
    params_.camBoundNormals_.push_back(camBoundNormals);
  }

  // Load mesh from STL file if provided.
  std::string ns = ros::this_node::getName();
  std::string stlPath = "";
  mesh_ = NULL;
  if (ros::param::get(ns + "/stl_file_path", stlPath)) {
    if (stlPath.length() > 0) {
      if (ros::param::get(ns + "/mesh_resolution", params_.meshResolution_)) {
        std::fstream stlFile;
        stlFile.open(stlPath.c_str());
        if (stlFile.is_open()) {
          mesh_ = new mesh::StlMesh(stlFile);
          mesh_->setResolution(params_.meshResolution_);
          mesh_->setOctomapManager(manager_);
          mesh_->setCameraParams(params_.camPitch_, params_.camHorizontal_, params_.camVertical_,
                                 params_.gainRange_);
        } else {
          ROS_WARN("Unable to open STL file");
        }
      } else {
        ROS_WARN("STL mesh file path specified but mesh resolution parameter missing!");
      }
    }
  }
  nh_private_.param("use_predictive_ig", use_predictive_ig_, false);
  nh_private_.param("compute_both_ig_trajectories", compute_both_ig_trajectories_, false);
  nh_private.param("max_planner_wait_time", max_planner_wait_time_, 60.0);

  scene_completion_interface_ = std::make_unique<SceneCompletion3dInterface>();

  initializeTree();

  // Not yet ready. Needs a position message first.
  ready_ = false;
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::initializeTree()
{
  // Initialize the tree instance(s).
  rrt_tree_ = std::make_shared<RrtTree>(mesh_, manager_);
  rrt_tree_->setParams(params_);

  peerPosClient1_ = nh_.subscribe("peer_pose_1", 10,
                                  &nbvInspection::RrtTree::setPeerStateFromPoseMsg1, rrt_tree_.get());
  peerPosClient2_ = nh_.subscribe("peer_pose_2", 10,
                                  &nbvInspection::RrtTree::setPeerStateFromPoseMsg2, rrt_tree_.get());
  peerPosClient3_ = nh_.subscribe("peer_pose_3", 10,
                                  &nbvInspection::RrtTree::setPeerStateFromPoseMsg3, rrt_tree_.get());
  // Subscribe to topic used for the collaborative collision avoidance (don't hit your peer).
  evadeClient_ = nh_.subscribe("/evasionSegment", 10, &nbvInspection::TreeBase<stateVec>::evade,
                               rrt_tree_.get());
}

template<typename stateVec>
nbvInspection::nbvPlanner<stateVec>::~nbvPlanner()
{
  if (manager_) {
    delete manager_;
  }
  if (mesh_) {
    delete mesh_;
  }
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::posCallback(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  rrt_tree_->setStateFromPoseMsg(pose);
  // Planner is now ready to plan.
  ready_ = true;
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::odomCallback(
    const nav_msgs::Odometry& pose)
{
  rrt_tree_->setStateFromOdometryMsg(pose);
  // Planner is now ready to plan.
  ready_ = true;
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::plannerCallback(nbvplanner::nbvp_srv::Request& req,
                                                          nbvplanner::nbvp_srv::Response& res)
{
  ros::Time computationTime = ros::Time::now();
  // Check that planner is ready to compute path.
  if (!ros::ok()) {
    ROS_INFO_THROTTLE(1, "Exploration finished. Not planning any further moves.");
    return true;
  }

  if (!ready_) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: Planner not ready!");
    return true;
  }
  if (manager_ == NULL) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
    return true;
  }
  if (manager_->getMapSize().norm() <= 0.0) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: Octomap is empty!");
    return true;
  }

  double original_gain = 0;
  std::vector<Eigen::Vector3d> original_gain_nodes;
  double predictive_gain = 0;
  std::vector<Eigen::Vector3d> predictive_gain_nodes;

  BestPathState path_state;
  if (use_predictive_ig_) {
    std::shared_ptr<volumetric_mapping::OctomapManager>
        predicted_map_manager(nullptr);
    getCompletedOcTreeManager(predicted_map_manager);
    rrt_tree_->setPredictedOctomapManager(predicted_map_manager);

    path_state = updateTree(InfoGainType::PREDICTIVE);
    res.predictive_path_success = (path_state != BestPathState::NOT_FOUND);
    getBestPath(
        InfoGainType::PREDICTIVE, req.header.frame_id, res.predictive_path,
        &predictive_gain, &predictive_gain_nodes);
  } else {
    scene_completion_interface_->updateInputOcTree(manager_->getOctree());
    path_state = updateTree(InfoGainType::ORIGINAL);
    res.original_path_success = (path_state != BestPathState::NOT_FOUND);
  }

  if (!use_predictive_ig_ || compute_both_ig_trajectories_) {
    const auto local_path_state = getBestPath(
        InfoGainType::ORIGINAL, req.header.frame_id, res.original_path,
        &original_gain, &original_gain_nodes);
    res.original_path_success = (local_path_state != BestPathState::NOT_FOUND);
  }

  if (path_state == BestPathState::OKAY) {
    const auto &path_to_execute =
        use_predictive_ig_ ? res.predictive_path : res.original_path;

    // Publish path to block for other agents (multi agent only).
    multiagent_collision_check::Segment segment;
    segment.header.stamp = ros::Time::now();
    segment.header.frame_id = params_.navigationFrame_;
    if (!path_to_execute.empty()) {
      segment.poses.push_back(path_to_execute.front());
      segment.poses.push_back(path_to_execute.back());
    }
    evadePub_.publish(segment);
  }

  ROS_INFO("Path computation lasted %2.3fs",
           (ros::Time::now() - computationTime).toSec());
  return true;
}

template<typename stateVec>
nbvInspection::BestPathState
nbvInspection::nbvPlanner<stateVec>::updateTree(const InfoGainType gain_type)
{
  const auto gainFound =
    (gain_type == InfoGainType::ORIGINAL)
      ? std::bind(&RrtTree::originalGainFound, rrt_tree_.get())
      : std::bind(&RrtTree::predictiveGainFound, rrt_tree_.get());

  // Clear old tree and reinitialize.
  rrt_tree_->clear();
  rrt_tree_->initialize();

  // Iterate the tree construction method.
  int loopCount = 0;
  double startTime = ros::Time::now().toSec();
  while ((!gainFound() || rrt_tree_->getCounter() < params_.initIterations_)
         && ros::Time::now().toSec() - startTime < max_planner_wait_time_
         && ros::ok()) {
    if (rrt_tree_->getCounter() > params_.cuttoffIterations_) {
      ROS_ERROR("No gain found until cutoff iterations, requesting shutdown");
      // ros::shutdown();
      return BestPathState::NOT_FOUND;
    }
    // if (loopCount > (unsigned long)100000 * (rrt_tree_->getCounter() + 1)) {
    //   ROS_ERROR_THROTTLE(
    //       1, "Exceeding maximum failed iterations, return to previous point!");
    //   return BestPathState::ALMOST_OKAY;
    // }
    rrt_tree_->iterate(1);
    loopCount++;
  }

  if (gainFound()) {
    rrt_tree_->memorizeBestBranch();
    return BestPathState::OKAY;
  } else {
    ROS_ERROR("No gain found until timeout, return to previous point!");
    return BestPathState::ALMOST_OKAY;
  }
}

template<typename stateVec>
nbvInspection::BestPathState
nbvInspection::nbvPlanner<stateVec>::getBestPath(
    InfoGainType gain_type,
    const std::string &frame_id,
    std::vector<geometry_msgs::Pose> &path,
    double * const gain/*=nullptr*/,
    std::vector<Eigen::Vector3d> * const gain_nodes/*=nullptr*/)
{
  path.clear();

  const auto gainFound =
    (gain_type == InfoGainType::ORIGINAL)
      ? std::bind(&RrtTree::originalGainFound, rrt_tree_.get())
      : std::bind(&RrtTree::predictiveGainFound, rrt_tree_.get());

  if (gainFound()) {
    // Extract the best edge.
    path = rrt_tree_->getBestEdge(gain_type, frame_id, gain, gain_nodes);
    return BestPathState::OKAY;
  } else if (rrt_tree_->getCounter() > params_.cuttoffIterations_) {
    ROS_ERROR("No gain found, requesting shutdown");
    // ros::shutdown();
    return BestPathState::NOT_FOUND;
  } else {
    // ROS_WARNING_THROTTLE(
    //     1, "Exceeding maximum failed iterations, return to previous point!");
    path = rrt_tree_->getPathBackToPrevious(frame_id, gain, gain_nodes);
    return BestPathState::ALMOST_OKAY;
  }
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::getCompletedOcTree(
    scene_completion_3d_interface::OcTreeTPtr &completed_octree) const
{
  if (!scene_completion_interface_) {
    ROS_ERROR("[getCompleteScene] scene completion interface uninitialized");
    return false;
  }
  if (!manager_) {
    ROS_ERROR("[getCompleteScene] manager uninitialized");
    return false;
  }

  const auto input_octree = manager_->getOctree();
  completed_octree = nullptr;
  sensor_msgs::PointCloud2 completed_points;

  auto success = scene_completion_interface_->completeScene(
    input_octree, completed_octree, completed_points
  );

  return success;
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::getCompletedOcTreeManager(
      std::shared_ptr<volumetric_mapping::OctomapManager>
          &completed_octree_manger) const
{
  scene_completion_3d_interface::OcTreeTPtr completed_octree(nullptr);
  if (getCompletedOcTree(completed_octree) && completed_octree) {
    completed_octree_manger =
        std::make_shared<volumetric_mapping::OctomapManager>(
            nh_, nh_private_, completed_octree,
            /*subscribe_topics=*/false);
    completed_octree_manger->setUnmappableKeys(manager_->getUnmappableKeys());
    return true;
  } else {
    return false;
  }
}

template<typename stateVec>
bool nbvInspection::nbvPlanner<stateVec>::setParams()
{
  std::string ns = ros::this_node::getName();
  bool ret = true;
  params_.v_max_ = 0.25;
  if (!ros::param::get(ns + "/system/v_max", params_.v_max_)) {
    ROS_WARN("No maximal system speed specified. Looking for %s. Default is 0.25.",
             (ns + "/system/v_max").c_str());
  }
  params_.dyaw_max_ = 0.5;
  if (!ros::param::get(ns + "/system/dyaw_max", params_.dyaw_max_)) {
    ROS_WARN("No maximal yaw speed specified. Looking for %s. Default is 0.5.",
             (ns + "/system/yaw_max").c_str());
  }
  params_.camPitch_ = {15.0};
  if (!ros::param::get(ns + "/system/camera/pitch", params_.camPitch_)) {
    ROS_WARN("No camera pitch specified. Looking for %s. Default is 15deg.",
             (ns + "/system/camera/pitch").c_str());
  }
  params_.camHorizontal_ = {90.0};
  if (!ros::param::get(ns + "/system/camera/horizontal", params_.camHorizontal_)) {
    ROS_WARN("No camera horizontal opening specified. Looking for %s. Default is 90deg.",
             (ns + "/system/camera/horizontal").c_str());
  }
  params_.camVertical_ = {60.0};
  if (!ros::param::get(ns + "/system/camera/vertical", params_.camVertical_)) {
    ROS_WARN("No camera vertical opening specified. Looking for %s. Default is 60deg.",
             (ns + "/system/camera/vertical").c_str());
  }
  if(params_.camPitch_.size() != params_.camHorizontal_.size() ||params_.camPitch_.size() != params_.camVertical_.size() ){
    ROS_WARN("Specified camera fields of view unclear: Not all parameter vectors have same length! Setting to default.");
    params_.camPitch_.clear();
    params_.camPitch_ = {15.0};
    params_.camHorizontal_.clear();
    params_.camHorizontal_ = {90.0};
    params_.camVertical_.clear();
    params_.camVertical_ = {60.0};
  }
  params_.igProbabilistic_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/probabilistic", params_.igProbabilistic_)) {
    ROS_WARN(
        "No gain coefficient for probability of cells specified. Looking for %s. Default is 0.0.",
        (ns + "/nbvp/gain/probabilistic").c_str());
  }
  params_.igFree_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/free", params_.igFree_)) {
    ROS_WARN("No gain coefficient for free cells specified. Looking for %s. Default is 0.0.",
             (ns + "/nbvp/gain/free").c_str());
  }
  params_.igOccupied_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/occupied", params_.igOccupied_)) {
    ROS_WARN("No gain coefficient for occupied cells specified. Looking for %s. Default is 0.0.",
             (ns + "/nbvp/gain/occupied").c_str());
  }
  params_.igUnmapped_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/gain/unmapped", params_.igUnmapped_)) {
    ROS_WARN("No gain coefficient for unmapped cells specified. Looking for %s. Default is 1.0.",
             (ns + "/nbvp/gain/unmapped").c_str());
  }
  params_.igArea_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/gain/area", params_.igArea_)) {
    ROS_WARN("No gain coefficient for mesh area specified. Looking for %s. Default is 1.0.",
             (ns + "/nbvp/gain/area").c_str());
  }
  params_.degressiveCoeff_ = 0.25;
  if (!ros::param::get(ns + "/nbvp/gain/degressive_coeff", params_.degressiveCoeff_)) {
    ROS_WARN(
        "No degressive factor for gain accumulation specified. Looking for %s. Default is 0.25.",
        (ns + "/nbvp/gain/degressive_coeff").c_str());
  }
  params_.extensionRange_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/tree/extension_range", params_.extensionRange_)) {
    ROS_WARN("No value for maximal extension range specified. Looking for %s. Default is 1.0m.",
             (ns + "/nbvp/tree/extension_range").c_str());
  }
  params_.initIterations_ = 15;
  if (!ros::param::get(ns + "/nbvp/tree/initial_iterations", params_.initIterations_)) {
    ROS_WARN("No number of initial tree iterations specified. Looking for %s. Default is 15.",
             (ns + "/nbvp/tree/initial_iterations").c_str());
  }
  params_.dt_ = 0.1;
  if (!ros::param::get(ns + "/nbvp/dt", params_.dt_)) {
    ROS_WARN("No sampling time step specified. Looking for %s. Default is 0.1s.",
             (ns + "/nbvp/dt").c_str());
  }
  params_.gainRange_ = 1.0;
  if (!ros::param::get(ns + "/nbvp/gain/range", params_.gainRange_)) {
    ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.",
             (ns + "/nbvp/gain/range").c_str());
  }
  if (!ros::param::get(ns + "/bbx/minX", params_.minX_)) {
    ROS_WARN("No x-min value specified. Looking for %s", (ns + "/bbx/minX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/minY", params_.minY_)) {
    ROS_WARN("No y-min value specified. Looking for %s", (ns + "/bbx/minY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/minZ", params_.minZ_)) {
    ROS_WARN("No z-min value specified. Looking for %s", (ns + "/bbx/minZ").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxX", params_.maxX_)) {
    ROS_WARN("No x-max value specified. Looking for %s", (ns + "/bbx/maxX").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxY", params_.maxY_)) {
    ROS_WARN("No y-max value specified. Looking for %s", (ns + "/bbx/maxY").c_str());
    ret = false;
  }
  if (!ros::param::get(ns + "/bbx/maxZ", params_.maxZ_)) {
    ROS_WARN("No z-max value specified. Looking for %s", (ns + "/bbx/maxZ").c_str());
    ret = false;
  }
  params_.softBounds_ = false;
  if (!ros::param::get(ns + "/bbx/softBounds", params_.softBounds_)) {
    ROS_WARN(
        "Not specified whether scenario bounds are soft or hard. Looking for %s. Default is false",
        (ns + "/bbx/softBounds").c_str());
  }
  params_.boundingBox_[0] = 0.5;
  if (!ros::param::get(ns + "/system/bbx/x", params_.boundingBox_[0])) {
    ROS_WARN("No x size value specified. Looking for %s. Default is 0.5m.",
             (ns + "/system/bbx/x").c_str());
  }
  params_.boundingBox_[1] = 0.5;
  if (!ros::param::get(ns + "/system/bbx/y", params_.boundingBox_[1])) {
    ROS_WARN("No y size value specified. Looking for %s. Default is 0.5m.",
             (ns + "/system/bbx/y").c_str());
  }
  params_.boundingBox_[2] = 0.3;
  if (!ros::param::get(ns + "/system/bbx/z", params_.boundingBox_[2])) {
    ROS_WARN("No z size value specified. Looking for %s. Default is 0.3m.",
             (ns + "/system/bbx/z").c_str());
  }
  params_.cuttoffIterations_ = 200;
  if (!ros::param::get(ns + "/nbvp/tree/cuttoff_iterations", params_.cuttoffIterations_)) {
    ROS_WARN("No cuttoff iterations value specified. Looking for %s. Default is 200.",
             (ns + "/nbvp/tree/cuttoff_iterations").c_str());
  }
  params_.zero_gain_ = 0.0;
  if (!ros::param::get(ns + "/nbvp/gain/zero", params_.zero_gain_)) {
    ROS_WARN("No zero gain value specified. Looking for %s. Default is 0.0.",
             (ns + "/nbvp/gain/zero").c_str());
  }
  params_.dOvershoot_ = 0.5;
  if (!ros::param::get(ns + "/system/bbx/overshoot", params_.dOvershoot_)) {
    ROS_WARN(
        "No estimated overshoot value for collision avoidance specified. Looking for %s. Default is 0.5m.",
        (ns + "/system/bbx/overshoot").c_str());
  }
  params_.log_ = false;
  if (!ros::param::get(ns + "/nbvp/log/on", params_.log_)) {
    ROS_WARN("Logging is off by default. Turn on with %s: true", (ns + "/nbvp/log/on").c_str());
  }
  params_.log_throttle_ = 0.5;
  if (!ros::param::get(ns + "/nbvp/log/throttle", params_.log_throttle_)) {
    ROS_WARN("No throttle time for logging specified. Looking for %s. Default is 0.5s.",
             (ns + "/nbvp/log/throttle").c_str());
  }
  params_.navigationFrame_ = "world";
  if (!ros::param::get(ns + "/tf_frame", params_.navigationFrame_)) {
    ROS_WARN("No navigation frame specified. Looking for %s. Default is 'world'.",
             (ns + "/tf_frame").c_str());
  }
  params_.pcl_throttle_ = 0.333;
  if (!ros::param::get(ns + "/pcl_throttle", params_.pcl_throttle_)) {
    ROS_WARN(
        "No throttle time constant for the point cloud insertion specified. Looking for %s. Default is 0.333.",
        (ns + "/pcl_throttle").c_str());
  }
  params_.inspection_throttle_ = 0.25;
  if (!ros::param::get(ns + "/inspection_throttle", params_.inspection_throttle_)) {
    ROS_WARN(
        "No throttle time constant for the inspection view insertion specified. Looking for %s. Default is 0.1.",
        (ns + "/inspection_throttle").c_str());
  }
  params_.exact_root_ = true;
  if (!ros::param::get(ns + "/nbvp/tree/exact_root", params_.exact_root_)) {
    ROS_WARN("No option for exact root selection specified. Looking for %s. Default is true.",
             (ns + "/nbvp/tree/exact_root").c_str());
  }
  return ret;
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTf(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  static double last = ros::Time::now().toSec();
  if (last + params_.pcl_throttle_ < ros::Time::now().toSec()) {
    rrt_tree_->insertPointcloudWithTf(pointcloud);
    last += params_.pcl_throttle_;
  }
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTfCamUp(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  static double last = ros::Time::now().toSec();
  if (last + params_.pcl_throttle_ < ros::Time::now().toSec()) {
    rrt_tree_->insertPointcloudWithTf(pointcloud);
    last += params_.pcl_throttle_;
  }
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::insertPointcloudWithTfCamDown(
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
  static double last = ros::Time::now().toSec();
  if (last + params_.pcl_throttle_ < ros::Time::now().toSec()) {
    rrt_tree_->insertPointcloudWithTf(pointcloud);
    last += params_.pcl_throttle_;
  }
}

template<typename stateVec>
void nbvInspection::nbvPlanner<stateVec>::evasionCallback(
    const multiagent_collision_check::Segment& segmentMsg)
{
  rrt_tree_->evade(segmentMsg);
}

template<typename stateVec>
Eigen::Vector3d nbvInspection::nbvPlanner<stateVec>::getMapSize()
{
    Eigen::Vector3d mapSize = Eigen::Vector3d::Zero();
    if (manager_) {
        mapSize = manager_->getMapSize();
    } else {
        ROS_WARN("Octomap manager not initialized");
    }
    return mapSize;
}

#endif // NBVP_HPP_
