/**
 * @file utils.cpp
 * Author: Rakesh Shrestha, rakeshs@sfu.ca
 */

#include <cstdlib>
#include <nbvplanner_tests/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace nbvplanner_tests
{
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
} // endnamespace nbvplanner_tests

