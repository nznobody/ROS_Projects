/*
 * Stereo2SensorProcessor.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Hannes Keller
 */

#include <chrono> //For timing

#include <elevation_mapping/sensor_processors/Stereo2SensorProcessor.hpp>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <vector>

namespace elevation_mapping {

Stereo2SensorProcessor::Stereo2SensorProcessor(ros::NodeHandle & nodeHandle, tf::TransformListener & transformListener)
    : SensorProcessorBase(nodeHandle, transformListener)
{

}

Stereo2SensorProcessor::~Stereo2SensorProcessor() {}

bool Stereo2SensorProcessor::readParameters()
{
  SensorProcessorBase::readParameters();
	nodeHandle_.param("sensor_processor/lateral_factor", sensorParameters_["lateral_factor"], 0.05);
	nodeHandle_.param("sensor_processor/normal_factor", sensorParameters_["normal_factor"], 1.0);
  nodeHandle_.param("sensor_processor/depth_to_disparity_factor", sensorParameters_["depth_to_disparity_factor"], 40.0);
  nodeHandle_.param("robot_base_frame_id", robotBaseFrameId_, std::string("/base_link"));
  nodeHandle_.param("map_frame_id", mapFrameId_, std::string("/map"));

	nodeHandle_.param("sensor_processor/cutoff_min_depth", sensorParameters_["cutoff_min_depth"], 1.0);
	nodeHandle_.param("sensor_processor/cutoff_max_depth", sensorParameters_["cutoff_max_depth"], 5.0);
	nodeHandle_.param("sensor_processor/down_sample_radius", sensorParameters_["down_sample_radius"], 0.025);
	nodeHandle_.param("sensor_processor/neighbour_radius", sensorParameters_["neighbour_radius"], 0.25);
	nodeHandle_.param("sensor_processor/min_neighbours", sensorParameters_["min_neighbours"], 5.0);
	
  double minUpdateRate;
  nodeHandle_.param("min_update_rate", minUpdateRate, 2.0);
  transformListenerTimeout_.fromSec(1.0 / minUpdateRate);
  ROS_ASSERT(!transformListenerTimeout_.isZero());

  return true;
}


bool Stereo2SensorProcessor::cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempPointCloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	//Testing filter
	//pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	int previous_count = static_cast<int>(pointCloud->size());
	
	originalWidth_ = pointCloud->width;
	auto start = std::chrono::steady_clock::now();
//	pcl::removeNaNFromPointCloud(*pointCloud, *pointCloud, indices_);
	auto mid = std::chrono::steady_clock::now();
	pointCloud->is_dense = true;
	//pointCloud->swap(*tempPointCloud_ptr);
	
	int mid_count = static_cast<int>(pointCloud->size());
	
	// Create the filtering object
	if (sensorParameters_.at("cutoff_min_depth") == sensorParameters_.at("cutoff_max_depth"))
	{
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(pointCloud);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(sensorParameters_.at("cutoff_min_depth"), sensorParameters_.at("cutoff_max_depth"));
		//pass.setFilterLimitsNegative (true);
		pass.filter(*pointCloud);
	}
	
	auto mid2 = std::chrono::steady_clock::now();
	int mid_count2 = static_cast<int>(pointCloud->size());
	
	//Downsample:
	if (sensorParameters_.at("down_sample_radius"))
	{
		pcl::UniformSampling<pcl::PointXYZRGB> uor;
		uor.setInputCloud(pointCloud);
		uor.setRadiusSearch(sensorParameters_.at("down_sample_radius"));
		uor.filter(*pointCloud);
	}
	
	auto mid3 = std::chrono::steady_clock::now();
	int mid_count3 = static_cast<int>(pointCloud->size());
	
	//Radius outlier removal
	if (sensorParameters_.at("min_neighbours") && sensorParameters_.at("neighbour_radius"))
	{
		pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
		ror.setInputCloud(pointCloud);
		ror.setRadiusSearch(sensorParameters_.at("neighbour_radius"));
		ror.setMinNeighborsInRadius(sensorParameters_.at("min_neighbours"));
		ror.filter(*pointCloud);
	}
	
	auto end = std::chrono::steady_clock::now();
	
	auto diff1 = mid - start;
	auto diff2 = mid2 - mid;
	auto diff3 = mid3 - mid2;
	auto diff4 = end - mid3;
	
	double time1 = std::chrono::duration <double, std::milli> (diff1).count();
	double time2 = std::chrono::duration <double, std::milli> (diff2).count();
	double time3 = std::chrono::duration <double, std::milli> (diff3).count();
	double time4 = std::chrono::duration <double, std::milli> (diff4).count();

	ROS_DEBUG("ElevationMap: %i	%i	%i	%i	%i		%f	%f	%f	%f", previous_count, mid_count, mid_count2, mid_count3, static_cast<int>(pointCloud->size()),time1, time2, time3, time4);
	return true;
}

bool Stereo2SensorProcessor::computeVariances(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
    const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
    Eigen::VectorXf& variances)
{
  variances.resize(pointCloud->size());

  // Projection vector (P).
  const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

  // Sensor Jacobian (J_s).
  const Eigen::RowVector3f sensorJacobian = projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

  // Robot rotation covariance matrix (Sigma_q).
  Eigen::Matrix3f rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

  // Preparations for#include <pcl/common/transforms.h> robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
  const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
  const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
  const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
  const Eigen::Matrix3f B_r_BS_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

	auto start = std::chrono::steady_clock::now();
	
  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    // For every point in point cloud.

    // Preparation.
    pcl::PointXYZRGB point = pointCloud->points[i];
    double disparity = abs(sensorParameters_.at("depth_to_disparity_factor")/point.z);
    Eigen::Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
    float heightVariance = 0.0; // sigma_p

	//Debugging
	//double test1 = sensorParameters_.at("depth_to_disparity_factor");
	//double test2 = sensorParameters_.at("depth_to_disparity_factor");
	//double test3 = sensorParameters_.at("normal_factor");
	//double test4 = sensorParameters_.at("lateral_factor");

    // Measurement distance.
    float measurementDistance = pointVector.norm();

    // Compute sensor covariance matrix (Sigma_S) with sensor model.
	  //Normal implements equation from thesis with d = 1

	//debugging

	//double bottom = (4 * (sensorParameters_.at("depth_to_disparity_factor") + measurementDistance));
	//double top = pow(measurementDistance, 2);

	  float varianceNormal = pow(pow(measurementDistance, 2) / (4 * (sensorParameters_.at("depth_to_disparity_factor") + measurementDistance)), 2) * sensorParameters_.at("normal_factor");
	  float varianceLateral = measurementDistance * sensorParameters_.at("lateral_factor");
	  
	  Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
	  sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

    // Robot rotation Jacobian (J_q).
    const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
    Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

    // Measurement variance for map (error propagation law).
    heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
    heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

    // Copy to list.
    variances(i) = heightVariance;
  }
	
	auto end = std::chrono::steady_clock::now();
	auto diff1 = end - start;
	double time1 = std::chrono::duration <double, std::milli> (diff1).count();
	ROS_DEBUG("ElevationMap/varianceCalc: %i	%f", static_cast<int>(pointCloud->size()), time1);

  return true;
}

int Stereo2SensorProcessor::getI(int index)
{
	assert(1);
  return indices_[index]/originalWidth_;
}

int Stereo2SensorProcessor::getJ(int index)
{
	assert(1);
  return indices_[index]%originalWidth_;
}

} /* namespace */

