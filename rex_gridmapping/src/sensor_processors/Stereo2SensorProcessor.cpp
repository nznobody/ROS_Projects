/*
 * Stereo2SensorProcessor.cpp
 *
 *  Created on: July 14, 2016
 *      Author: Manu Lange
 */

#include <rex_gridmapping/sensor_processors/Stereo2SensorProcessor.hpp>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
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
		nodeHandle_.param("sensor_processor/base_line", sensorParameters_["base_line"], 0.0);
		nodeHandle_.param("sensor_processor/focal_length", sensorParameters_["focal_length"], 0.0);
		nodeHandle_.param("sensor_processor/matching_error", sensorParameters_["matching_error"], 0.0);
		nodeHandle_.param("sensor_processor/pointing_error", sensorParameters_["pointing_error"], 0.0);
		nodeHandle_.param("sensor_processor/cutoff_min_depth", sensorParameters_["cutoff_min_depth"], std::numeric_limits<double>::min());
		nodeHandle_.param("sensor_processor/cutoff_max_depth", sensorParameters_["cutoff_max_depth"], std::numeric_limits<double>::max());
		nodeHandle_.param("robot_base_frame_id", robotBaseFrameId_, std::string("/robot"));
		nodeHandle_.param("map_frame_id", mapFrameId_, std::string("/map"));

		double minUpdateRate;
		nodeHandle_.param("min_update_rate", minUpdateRate, 2.0);
		transformListenerTimeout_.fromSec(1.0 / minUpdateRate);
		ROS_ASSERT(!transformListenerTimeout_.isZero());

		return true;
	}


	bool Stereo2SensorProcessor::cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
	{
//		pcl::PointCloud<pcl::PointXYZRGB> tempPointCloud;
//
//		originalWidth_ = pointCloud->width;
//		pcl::removeNaNFromPointCloud(*pointCloud, tempPointCloud, indices_);
//		tempPointCloud.is_dense = true;
//		pointCloud->swap(tempPointCloud);
		
		pcl::PassThrough<pcl::PointXYZRGB> passThroughFilter;
		pcl::PointCloud<pcl::PointXYZRGB> tempPointCloud;

		passThroughFilter.setInputCloud(pointCloud);
		passThroughFilter.setFilterFieldName("z");
		passThroughFilter.setFilterLimits(-1.0, 5.0);
		// This makes the point cloud also dense (no NaN points).
		passThroughFilter.filter(tempPointCloud);
		tempPointCloud.is_dense = true;
		pointCloud->swap(tempPointCloud);

		ROS_DEBUG("ElevationMap: cleanPointCloud() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
		return true;
	}

	bool Stereo2SensorProcessor::computeVariances(
	    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
		const Eigen::Matrix<double,
		6,
		6>& robotPoseCovariance,
		Eigen::VectorXf& variances)
	{
		variances.resize(pointCloud->size());
		
			// Const values for error calcs
		//const float normalFactor = sensorParameters_["matching_error"] / (sensorParameters_["base_line"] * sensorParameters_["focal_length"]); //Negative not needed, its abs.
		//const float lateralFactor = sensorParameters_["pointing_error"] / sensorParameters_["focal_length"];

			// Projection vector (P).
		const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

			// Sensor Jacobian (J_s).
		const Eigen::RowVector3f sensorJacobian = projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

			// Robot rotation covariance matrix (Sigma_q).
		Eigen::Matrix3f rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

			// Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
		const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
		const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
		const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
		const Eigen::Matrix3f B_r_BS_skew = kindr::linear_algebra::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

		for (unsigned int i = 0; i < pointCloud->size(); ++i)
		{
			// For every point in point cloud.

					// Preparation.
			auto& point = pointCloud->points[i];
			Eigen::Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
			float heightVariance = 0.0; // sigma_p

					// Measurement distance.
			float measurementDistance = pointVector.norm();

					// Compute sensor covariance matrix (Sigma_S) with sensor model. According to PtGreys method: https://www.ptgrey.com/KB/10589
//			float varianceNormal = pow(normalFactor * pow(measurementDistance, 2), 2);
//			float varianceLateral = pow(lateralFactor * measurementDistance, 2);
//			
//			Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
//			sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;
//
//					// Robot rotation Jacobian (J_q).
//			const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::linear_algebra::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
//			Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);
//
//					// Measurement variance for map (error propagation law).
//			heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
//			heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();
			
			//Testing simple model. variance = intercept + d*const
			heightVariance = sensorParameters_["matching_error"] + (sensorParameters_["pointing_error"] * measurementDistance);

					// Copy to list.
			variances(i) = heightVariance;
		}

		return true;
	}

	int Stereo2SensorProcessor::getI(int index)
	{
		return indices_[index] / originalWidth_;
	}

	int Stereo2SensorProcessor::getJ(int index)
	{
		return indices_[index] % originalWidth_;
	}

} /* namespace */

