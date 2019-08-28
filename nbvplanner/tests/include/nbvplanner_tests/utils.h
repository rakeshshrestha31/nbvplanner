/**
 * @file utils.h
 * Author: Rakesh Shrestha, rakeshs@sfu.ca
 */

#ifndef NBVPLANNER_TESTS_UTILS_H_
#define NBVPLANNER_TESTS_UTILS_H_

#include <vector>
#include <Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace nbvplanner_tests
{
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
} // endnamespace nbvplanner_tests

#endif // NBVPLANNER_TESTS_UTILS_H_

