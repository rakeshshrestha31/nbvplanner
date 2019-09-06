/**
 * @file predicted_gain_viz.cpp
 * Author: Rakesh Shrestha, rakeshs@sfu.ca
 */

#include <ctime>
#include <memory>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <octomap_world/octomap_manager.h>

#include <nbvplanner/nbvp.h>
#include <nbvplanner/nbvp.hpp>

#include <nbvplanner_tests/utils.h>

// custom includes
// directly use scene_completion_3d_interface as shared library
// instead of using ROS service interface
// #include <scene_completion_3d_msgs/GetCompleteScene.h>
#include <scene_completion_3d_interface/scene_completion_3d_interface.h>

using namespace nbvplanner_tests;

// free function
void publishVectorCloud(
    const std::vector<Eigen::Vector3d> &vector_cloud,
    ros::Publisher &point_cloud_pub,
    const std::string &tf_frame_id
);

/**
 * visualizing information gain with scene prediction
 */
class PredictedGainViz
{
public:
  PredictedGainViz();
  bool loadOctomapFile(const std::string &octomap_filename);
  void vizInfoGain();

  /** Get random (feasible) state */
  bool getRandomState(StateVecT &state_vec);

  /** Get RRT tree from NBV object */
  nbvInspection::RrtTree* getRrtTree();

  /** Publish state whose Info Gain (Ig) is being measured */
  void publishIgState(const StateVecT &state_vec);

  /** Publish nodes which contributed to Info Gain (Ig) */
  void publishIgNodes(const std::vector<Eigen::Vector3d> &gain_nodes);

  /** Publish nodes which contributed to Info Gain (Ig) from predicted map*/
  void publishIgPredictedNodes(const std::vector<Eigen::Vector3d> &gain_nodes);

  /** Get complete scene from the map created thus far */
  void getCompleteScene(
      scene_completion_3d_interface::OcTreeTPtr &_completed_octree
  );

protected:
  using SceneCompletion3dInterface =
      scene_completion_3d_interface::SceneCompletion3dInterface;

  std::unique_ptr< nbvInspection::nbvPlanner<StateVecT> > nbv_planner_;
  SceneCompletion3dInterface scene_completion_interface_;

  std::string tf_frame_id_;

  ros::NodeHandle global_nh_;
  ros::NodeHandle local_nh_;

  ros::Publisher ig_pose_publisher_;
  ros::Publisher ig_nodes_publisher_;
  ros::Publisher ig_predicted_nodes_publisher_;

}; //endclass PredictedGainViz

PredictedGainViz::PredictedGainViz()
  : local_nh_("~")
{
  ig_pose_publisher_ = local_nh_.advertise<geometry_msgs::PoseStamped>(
      "ig_pose", 5, true
  );
  ig_nodes_publisher_ = local_nh_.advertise<PointCloudT>(
      "ig_nodes", 5, true
  );
  ig_predicted_nodes_publisher_ = local_nh_.advertise<PointCloudT>(
      "ig_predicted_nodes", 5, true
  );

  local_nh_.param("tf_frame", tf_frame_id_, std::string("world"));

  std::string octomap_file;
  if (!local_nh_.getParam("octomap_file", octomap_file))
  {
    ROS_ERROR("octomap_file param not provided");
    exit(1);
  }

  nbv_planner_ = std::make_unique< nbvInspection::nbvPlanner<StateVecT> >(
      global_nh_, local_nh_
  );

  // no need to do this.
  // The OctomapManager constructor reads filename from the params and does it already
  // if (!loadOctomapFile(octomap_file))
  // {
  //   exit(1);
  // }

  vizInfoGain();
}

bool PredictedGainViz::loadOctomapFile(const std::string &filename)
{
  auto *octomap_manager = nbv_planner_->getOctomapManager();
  const auto file_extension = filename.substr(filename.length()-3, 3);
  if (file_extension == ".ot")
  {
    octomap_manager->loadOctomapFromFile(filename);
    return true;
  }
  else if (file_extension == ".bt")
  {
    octomap_manager->loadOctomapFromFileBinary(filename);
    return true;
  }
  else
  {
    ROS_ERROR("Invalid octomap filename extension");
    return false;
  }
}

bool PredictedGainViz::getRandomState(StateVecT &state_vec)
{
  // random orientation
  double random_orientation =  signedUnitRandom() * M_PI;

  // get random free point
  std::vector<std::pair<Eigen::Vector3d, double> > free_boxes;

  auto *octomap_manager = nbv_planner_->getOctomapManager();
  octomap_manager->getAllFreeBoxes(&free_boxes);

  if (free_boxes.empty())
  {
    ROS_WARN("No free voxels in octomap!!!");
    return false;
  }

  auto random_idx = rand() % free_boxes.size();
  auto random_point = free_boxes[random_idx].first;

  state_vec << random_point(0), random_point(1),
               random_point(2), random_orientation;

  return true;
}

void PredictedGainViz::publishIgState(const StateVecT &state_vec)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = tf_frame_id_;
  pose_stamped.header.stamp = ros::Time::now();
  stateVecToPose(state_vec, pose_stamped.pose);
  ig_pose_publisher_.publish(pose_stamped);
}

// free function
void publishVectorCloud(
    const std::vector<Eigen::Vector3d> &vector_cloud,
    ros::Publisher &point_cloud_pub,
    const std::string &tf_frame_id
)
{
  if (vector_cloud.empty())
  {
    return;
  }
  PointCloudT point_cloud;
  point_cloud.header.frame_id = tf_frame_id;
  pcl_conversions::toPCL(ros::Time::now(), point_cloud.header.stamp);
  vecVectorToPointCloud(vector_cloud, point_cloud);
  point_cloud_pub.publish(point_cloud);
}

void PredictedGainViz::publishIgNodes(
    const std::vector<Eigen::Vector3d> &gain_nodes
)
{
  return publishVectorCloud(gain_nodes, ig_nodes_publisher_, tf_frame_id_);
}

void PredictedGainViz::publishIgPredictedNodes(
    const std::vector<Eigen::Vector3d> &gain_nodes
)
{
  return publishVectorCloud(
      gain_nodes, ig_predicted_nodes_publisher_, tf_frame_id_
  );
}

void PredictedGainViz::getCompleteScene(
    scene_completion_3d_interface::OcTreeTPtr &_completed_octree
)
{
  const auto input_octree = nbv_planner_->getOctomapManager()->getOctree();
  _completed_octree = nullptr;
  sensor_msgs::PointCloud2 completed_points;

  auto success = scene_completion_interface_.completeScene(
    input_octree, _completed_octree, completed_points
  );
}

nbvInspection::RrtTree* PredictedGainViz::getRrtTree()
{
  auto tree = dynamic_cast<nbvInspection::RrtTree*>(
      nbv_planner_->tree_
  );
  if (!tree)
  {
    ROS_ERROR("Could not get RRT Tree");
  }
  return tree;
}

void PredictedGainViz::vizInfoGain()
{
  scene_completion_3d_interface::OcTreeTPtr completed_octree(nullptr);
  getCompleteScene(completed_octree);
  volumetric_mapping::OctomapManager predicted_octomap_manager(
      global_nh_, local_nh_, completed_octree, /*subscribe_topics=*/false
  );

  auto tree = getRrtTree();
  if (!tree)
  {
    return;
  }

  double gain = 0.0;
  double predicted_gain = 0.0;
  std::vector<Eigen::Vector3d> gain_nodes;
  std::vector<Eigen::Vector3d> predicted_gain_nodes;
  StateVecT random_state;
  // get a state with non-zero gain
  do
  {
    if (!getRandomState(random_state))
    {
      return;
    }
    gain = tree->gain(
        random_state, &gain_nodes, &predicted_octomap_manager,
        &predicted_gain_nodes, &predicted_gain
    );
  } while (gain == 0.0 && ros::ok());

  ROS_INFO_STREAM("random state: " << random_state.transpose());
  ROS_INFO("Gain: %f, Gain nodes: %lu", gain, gain_nodes.size());
  ROS_INFO(
      "Predicted Gain: %f, Gain nodes: %lu",
      predicted_gain, predicted_gain_nodes.size()
  );

  publishIgState(random_state);
  publishIgNodes(gain_nodes);
  publishIgPredictedNodes(predicted_gain_nodes);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "predicted_gain_viz");

  // random seed
  srand(time(NULL));

  PredictedGainViz gain_viz;

  ros::spin();
  return 0;
}
