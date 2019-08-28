/**
 * @file original_gain_viz.cpp
 * Author: Rakesh Shrestha, rakeshs@sfu.ca
 */

#include <cstdlib>
#include <ctime>
#include <memory>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nbvplanner/nbvp.h>
#include <nbvplanner/nbvp.hpp>

// typedefs
typedef Eigen::Vector4d StateVecT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

// free functions
/** random number from 0 to 1 */
double unsignedUnitRandom();

/** random number from -1 to 1 */
double signedUnitRandom();

/** state vector to ROS msg type */
void stateVecToPose(const StateVecT &state_vec, geometry_msgs::Pose &pose);

/** vector of eigen vectors to ROS point cloud type*/
void vecVectorToPointCloud(const std::vector<Eigen::Vector3d> &vecVector,
                           PointCloudT &point_cloud);

double unsignedUnitRandom()
{
  return (double)(rand()) / RAND_MAX;
}

double signedUnitRandom()
{
  return (unsignedUnitRandom() - 0.5) * 2;
}

void stateVecToPose(const StateVecT &state_vec, geometry_msgs::Pose &pose)
{
  tf2::Quaternion tf2_quaternion;
  tf2_quaternion.setRPY(0, 0, state_vec(3));
  tf2::convert(tf2_quaternion, pose.orientation);

  pose.position.x = state_vec(0);
  pose.position.y = state_vec(1);
  pose.position.z = state_vec(2);
}

void vecVectorToPointCloud(const std::vector<Eigen::Vector3d> &vecVector,
                           PointCloudT &point_cloud)
{
  point_cloud.width = 1;
  point_cloud.height = 1;
  point_cloud.points.clear();

  for (const auto &vec: vecVector)
  {
    point_cloud.push_back(
      pcl::PointXYZ(vec(0), vec(1), vec(2))
    );
  }
}

/**
 * visualizing original (without scene prediction) information gain
 */
class OriginalGainViz
{
public:
  OriginalGainViz();
  bool loadOctomapFile(const std::string &octomap_filename);
  void vizInfoGain();

  /** Get random (feasible) state */
  bool getRandomState(StateVecT &state_vec);

  /** Publish state whose Info Gain (Ig) is being measured */
  void publishIgState(const StateVecT &state_vec);

  /** Publish nodes which contributed to Info Gain (Ig) */
  void publishIgNodes(const std::vector<Eigen::Vector3d> &gain_nodes);

protected:
  std::unique_ptr< nbvInspection::nbvPlanner<StateVecT> > nbv_planner_;
  std::string tf_frame_id_;

  ros::NodeHandle global_nh_;
  ros::NodeHandle local_nh_;

  ros::Publisher ig_pose_publisher_;
  ros::Publisher ig_nodes_publisher_;
}; //endclass OriginalGainViz

OriginalGainViz::OriginalGainViz()
  : local_nh_("~")
{
  ig_pose_publisher_ = local_nh_.advertise<geometry_msgs::PoseStamped>(
      "ig_pose", 5, true
  );
  ig_nodes_publisher_ = local_nh_.advertise<PointCloudT>(
      "ig_nodes", 5, true
  );
  local_nh_.param("tf_frame", tf_frame_id_, std::string("world"));

  std::string octomap_file;
  if (!local_nh_.getParam("octomap_file", octomap_file))
  {
    ROS_ERROR("octomap_file param not provided");
    exit(1);
  }

  nbv_planner_.reset(
      new nbvInspection::nbvPlanner<StateVecT>(global_nh_, local_nh_)
  );

  // no need to do this.
  // The OctomapManager constructor reads filename from the params and does it already
  // if (!loadOctomapFile(octomap_file))
  // {
  //   exit(1);
  // }

  vizInfoGain();
}

bool OriginalGainViz::loadOctomapFile(const std::string &filename)
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

bool OriginalGainViz::getRandomState(StateVecT &state_vec)
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

void OriginalGainViz::publishIgState(const StateVecT &state_vec)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = tf_frame_id_;
  pose_stamped.header.stamp = ros::Time::now();
  stateVecToPose(state_vec, pose_stamped.pose);
  ig_pose_publisher_.publish(pose_stamped);
}

void OriginalGainViz::publishIgNodes(
    const std::vector<Eigen::Vector3d> &gain_nodes
)
{
  PointCloudT point_cloud;
  point_cloud.header.frame_id = tf_frame_id_;
  pcl_conversions::toPCL(ros::Time::now(), point_cloud.header.stamp);
  vecVectorToPointCloud(gain_nodes, point_cloud);
  ig_nodes_publisher_.publish(point_cloud);
}

void OriginalGainViz::vizInfoGain()
{
  auto tree = dynamic_cast<nbvInspection::RrtTree*>(
      nbv_planner_->tree_
  );
  if (!tree)
  {
    ROS_ERROR("Could not get RRT Tree");
    return;
  }

  double gain = 0.0;
  std::vector<Eigen::Vector3d> gain_nodes;
  StateVecT random_state;
  // get a state with non-zero gain
  do
  {
    if (!getRandomState(random_state))
    {
      return;
    }
    gain = tree->gain(random_state, &gain_nodes);
  } while (gain == 0.0 && ros::ok());

  ROS_INFO_STREAM("random state: " << random_state.transpose());
  ROS_INFO("Gain: %f, Gain nodes: %lu", gain, gain_nodes.size());
  publishIgState(random_state);
  publishIgNodes(gain_nodes);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "original_gain_viz");

  // random seed
  srand(time(NULL));

  OriginalGainViz gain_viz;

  ros::spin();
  return 0;
}