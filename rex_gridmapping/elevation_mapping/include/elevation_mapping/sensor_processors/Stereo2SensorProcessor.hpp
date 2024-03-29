/*
 * Stereo2SensorProcessor.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Hannes Keller
 */

#pragma once

#include <elevation_mapping/sensor_processors/SensorProcessorBase.hpp>

namespace elevation_mapping {

/*!
 * Sensor processor for stereo camera sensors.
 * Cleans the point cloud, transforms it to a desired frame, and
 * computes the measurement variances based on a sensor model in
 * the desired frame.
 *
 * "Localization and Path Planning of a Climbing Robot for Corrosion Monitoring", Hannes Keller, Semester Project, ETH Zurich, 2014.
 */

class Stereo2SensorProcessor: public SensorProcessorBase
{
public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param transformListener the ROS transform listener.
   */
  Stereo2SensorProcessor(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener);

  /*!
   * Destructor.
   */
  virtual ~Stereo2SensorProcessor();

private:

  /*!
   * Reads and verifies the parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Clean the point cloud. Points below the minimal and above the maximal sensor
   * cutoff value are dropped.
   * @param pointCloud the point cloud to clean.
   * @return true if successful.
   */
  virtual bool cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

  /*!
   * Computes the elevation map height variances for each point in a point cloud with the
   * sensor model and the robot pose covariance.
   * @param[in] pointCloud the point cloud for which the variances are computed.
   * @param[in] robotPoseCovariance the robot pose covariance matrix.
   * @param[out] variances the elevation map height variances.
   * @return true if successful.
   */
  virtual bool computeVariances(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
      const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
      Eigen::VectorXf& variances);

  //! Helper functions to get i-j indices out of a single index.
  int getI(int index);
  int getJ(int index);

  //! Stores 'original' point cloud indices of the cleaned point cloud.
  std::vector<int> indices_;
  int originalWidth_;
};

} /* namespace */
