//	========================================================================
//					Rex ROS Sample Core Node
//				Copyright(C) {2016}  {Manu Lange}
//
//	This program is free software : you can redistribute it and / or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	(at your option) any later version.
//
//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
//	GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with this program.If not, see < http ://www.gnu.org/licenses/>.
//	========================================================================
 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/uniform_sampling.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>

//tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <iostream>
#include <vector>
#include <mutex>
#include <chrono>
#include <thread>

#define SAMPLE_COUNT 10
#define DEBUG_MARKER 0
#define BUFFER_COUNT 30

bool	callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
void	calib_thread();
bool	AreQuaternionsClose(Eigen::Quaterniond q1, Eigen::Quaterniond q2);
Eigen::Quaterniond AverageQuaternion(Eigen::Vector4d &cumulative, Eigen::Quaterniond newRotation, Eigen::Quaterniond firstRotation, int addAmount);

struct Vec4f
{
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
	float w = 0.0;
};

//Global for thread sharing
geometry_msgs::TransformStamped transformStamped;
geometry_msgs::TransformStamped transformStamped_base;
std::mutex	tf_lock;
bool		b_update;
std::vector <Eigen::Quaterniond> samplesQuat;
std::vector <tf2::Vector3>		samplesVect;
std::vector <double>			samplesAngle;
std::thread t_callback;
ros::Publisher pubMark;

// This will take the cloud and calculate the transform required to align ground plane with XY plane of "/odom" frame
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
	if (!b_update)
		return;
	
//	//Ignore the first few clouds as they are often noisy whilst camera adjusts
//	static int ignore_counter = 0;
//	if (ignore_counter < BUFFER_COUNT)
//	{
//		ignore_counter++;
//		return;
//	}
	
	//Get a local node handle
	static ros::NodeHandle thread_nh("~");
	//Get variables from variable server
	static float floor_tolerance = -1.0;
	if (floor_tolerance < 0.0)	//If first round, get parameter
		thread_nh.getParam("floor_tolerance", floor_tolerance);
	//Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*input, *ptr_cloud);
	
	//Downsample
	size_t	pt_count = ptr_cloud->size();
	
	pcl::UniformSampling<pcl::PointXYZ> uor;
	uor.setInputCloud(ptr_cloud);
	uor.setRadiusSearch(0.025); //Hardcoded for now...
	uor.filter(*ptr_cloud);
	
	ROS_WARN("REX_GroundCalib: Downsample: %i	%i", pt_count, ptr_cloud->size());
	
	if (!ptr_cloud->size())
		return;
	
	pcl::ModelCoefficients coefficients;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// Create the segmentation object ==========================
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	if (floor_tolerance <= 0.0)
		floor_tolerance = 0.10; //defualt the value
	seg.setDistanceThreshold(floor_tolerance);

	//Do segmentation ==========================================
	seg.setInputCloud(ptr_cloud);
	seg.segment(*inliers, coefficients);
	
	//Use the output ===========================================
	//Use eigen to calculate points that make plane
	//	Eigen::Vector3d A((coefficients.values[0] / coefficients.values[2]), (coefficients.values[1] / coefficients.values[2]), (coefficients.values[3] / coefficients.values[2]));
	//Closest point on plane (is perpendicular):
	Eigen::Vector3d	perpendicular(-coefficients.values[0]*coefficients.values[3], -coefficients.values[1]*coefficients.values[3], -coefficients.values[2]*coefficients.values[3]);
	Eigen::Vector3d	zaxis(0, 0, -1);
	Eigen::Quaterniond	qangle = Eigen::Quaterniond().setFromTwoVectors(perpendicular, zaxis);
	samplesQuat.push_back(qangle);
	//This calculates the angle the transform needs to have to make plane flat!
	double angle = -atan2((coefficients.values[0]*coefficients.values[3]), (coefficients.values[2]*coefficients.values[3]));	//+ because atan2 disregards sign, should be negative if properly signed
	std::cout << "Angle is: " << angle << std::endl;
	samplesAngle.push_back(angle);
	
	//Save Offset Vector
	tf2::Vector3 temp(-coefficients.values[0]*coefficients.values[3], -coefficients.values[1]*coefficients.values[3], -coefficients.values[2]*coefficients.values[3]);
	
	samplesVect.push_back(temp);
	
	if (DEBUG_MARKER)
	{
		//Use eigen to calculate points that make plane
		Eigen::Vector3d A((coefficients.values[0] / coefficients.values[2]), (coefficients.values[1] / coefficients.values[2]), (coefficients.values[3] / coefficients.values[2]));
		//Eigen::Vector3f x(0, 0, 1);
		double z1 = -A.dot(Eigen::Vector3d(1, 0, 1));
		double z2 = -A.dot(Eigen::Vector3d(1, 1, 1));
		double z3 = -A.dot(Eigen::Vector3d(3, 1, 1));
		double z4 = -A.dot(Eigen::Vector3d(3, -1, 1));
		double z5 = -A.dot(Eigen::Vector3d(1, -1, 1));
	
		geometry_msgs::Point test1, test2, test3, test4, test5, perpen, origin;
		tf::pointEigenToMsg(Eigen::Vector3d(1, 0, z1), test1);
		tf::pointEigenToMsg(Eigen::Vector3d(1, 1, z2), test2);
		tf::pointEigenToMsg(Eigen::Vector3d(3, 1, z3), test3);
		tf::pointEigenToMsg(Eigen::Vector3d(3, -1, z4), test4);
		tf::pointEigenToMsg(Eigen::Vector3d(1, -1, z5), test5);
		tf::pointEigenToMsg(Eigen::Vector3d(-coefficients.values[0]*coefficients.values[3], -coefficients.values[1]*coefficients.values[3], -coefficients.values[2]*coefficients.values[3]), perpen);
		origin.x = origin.y = origin.z = 0;
	
		//Publish the marker for ground plane visualising
		visualization_msgs::Marker marker;
		marker.header.frame_id = input->header.frame_id;
		marker.header.stamp = ros::Time();
		marker.ns = "ground_calib";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.01;
		marker.scale.y = 1;
		marker.scale.z = 1;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.points.push_back(origin);
		marker.points.push_back(perpen);
		marker.points.push_back(test1);
		marker.points.push_back(test2);
		marker.points.push_back(test3);
		marker.points.push_back(test4);
		marker.points.push_back(test5);
		marker.points.push_back(test1);
		pubMark.publish(marker);
	}
}

int
main(int argc, char** argv)
{
  // Initialize ROS
	ros::init(argc, argv, "ground_calib");
	ros::NodeHandle nh("~");
	//Read arguments
	std::string input_cloud = "/camera/point_cloud/cloud", odom_frame = "/odom", camera_frame = "/zed_initial_frame";
	nh.getParam("input_cloud", input_cloud);
	nh.getParam("odom_frame", odom_frame);
	nh.getParam("camera_frame", camera_frame);
	
	b_update = false;

	// Create a ROS subscriber for the input point cloud
	//ros::Subscriber sub = nh.subscribe("/camera/point_cloud/cloud", 1, cloud_cb);
	//Create the service provider
	ros::ServiceServer service = nh.advertiseService("trigger_calib", callback);
	
	//Debug marker
	if (DEBUG_MARKER)
	{
		// Create a ROS publisher for the output model coefficients
		pubMark = nh.advertise<visualization_msgs::Marker>("marker", 1);
	}

	//Set up TF object
	tf2_ros::TransformBroadcaster transform_odom_broadcaster;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "/odom";
	transformStamped.child_frame_id = "/zed_initial_frame";
	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = 0.0;
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion quat;
	quat.setRPY(0.0, 0.0, 0.0);
	transformStamped.transform.rotation.x = quat.x();
	transformStamped.transform.rotation.y = quat.y();
	transformStamped.transform.rotation.z = quat.z();
	transformStamped.transform.rotation.w = quat.w();
	
		//Set up TF object for base_link
	//xyz transform needs to be converted from flat into zed _tracked frame...
	tf2::Vector3	baseOffset;
	tf2::Transform	baseTransform;
	baseTransform.setRotation(quat);
	transformStamped_base.header.stamp = ros::Time::now();
	transformStamped_base.header.frame_id = "/zed_tracked_frame";
	transformStamped_base.child_frame_id = "/base_link";
	transformStamped_base.transform.translation.x = 0.0;
	transformStamped_base.transform.translation.y = 0.0;
	transformStamped_base.transform.translation.z = 0.0;
	transformStamped_base.transform.rotation.x = quat.x();
	transformStamped_base.transform.rotation.y = quat.y();
	transformStamped_base.transform.rotation.z = quat.z();
	transformStamped_base.transform.rotation.w = quat.w();
	
	ros::Rate	loopRate(10);
	
	while (ros::ok())
	{
		tf_lock.lock();
		transformStamped.header.stamp = ros::Time::now();
		transform_odom_broadcaster.sendTransform(transformStamped);
		
		//Base_link
		transformStamped_base.header.stamp = ros::Time::now();
		transform_odom_broadcaster.sendTransform(transformStamped_base);
		tf_lock.unlock();
		
		//Spin and sleep at the end
		ros::spinOnce();
		loopRate.sleep();
	}
	
	ROS_WARN("Shutdown request received by ground_calib");
	ros::shutdown();
	return 0;
}

bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	if (t_callback.joinable())
		t_callback.join();
	t_callback = std::thread(calib_thread);
	return true;
}


//This function will get the calibrated floor angles and distances, then return. Can be used as a fire and forget method.
//Relays the information back via global variables
//TODO: Make it relay back via references / shared ptr
void	calib_thread()
{
	//Get a local node handle
	ros::NodeHandle thread_nh("~");
	//Get variables from variable server
	std::string input_cloud = "/camera/point_cloud/cloud", odom_frame = "/odom", camera_frame = "/zed_initial_frame";
	thread_nh.getParam("input_cloud", input_cloud);
	thread_nh.getParam("odom_frame", odom_frame);
	thread_nh.getParam("camera_frame", camera_frame);
	double offsetX, offsetY, offsetZ;
	thread_nh.param("base_offset_x", offsetX, 0.0);
	thread_nh.param("base_offset_y", offsetY, 0.0);
	thread_nh.param("base_offset_z", offsetZ, 0.0);
	
	//Clear globals
	samplesQuat.clear();
	samplesVect.clear();
	samplesAngle.clear();
	
	
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = thread_nh.subscribe(input_cloud, 1, cloud_cb);
	
	//Give the camera time to stabalise:
	ROS_WARN("Waiting for camera to stabalise");
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));	//Sleep
	//Enable processing
	b_update = true;
	
	while (samplesAngle.size() < SAMPLE_COUNT)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));	//Sleep
	}
	b_update = false;
	//Unsubscribe... to stop wasting cpu
	sub.shutdown();
	
	//Average Angle Samples
	//Worth a read: http://wiki.unity3d.com/index.php/Averaging_Quaternions_and_Vectors
	double	cumulativeAngle = 0.0;
	for (auto var : samplesAngle)
	{
		cumulativeAngle += var;
	}
	cumulativeAngle = cumulativeAngle / samplesAngle.size();
	tf2::Quaternion averagedQuat;
	averagedQuat.setRPY(0.0, (float)cumulativeAngle, 0.0);
	averagedQuat.normalize();
	//average quarts
	Eigen::Vector4d cumulatives;
	cumulatives.setZero();
	int counter = 0;
	Eigen::Quaterniond	runningAverage;
	for (auto var : samplesQuat)
	{
		counter++;
		runningAverage = AverageQuaternion(cumulatives, var, samplesQuat[0], counter);
	}
	
	//Average Vector Samples
	tf2::Vector3 cumulativeVect(0.0, 0.0, 0.0);
	for (auto var : samplesVect)
	{
		cumulativeVect += var;
	}
	cumulativeVect.m_floats[0] = cumulativeVect.m_floats[0] / samplesVect.size();
	cumulativeVect.m_floats[1] = cumulativeVect.m_floats[1] / samplesVect.size();
	cumulativeVect.m_floats[2] = cumulativeVect.m_floats[2] / samplesVect.size();
	
	//Update global transform object
	std::lock_guard<std::mutex> lock(tf_lock);
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "/odom";
	transformStamped.child_frame_id = "/zed_initial_frame";
	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = 0.0;
	transformStamped.transform.translation.z = cumulativeVect.length();
	transformStamped.transform.rotation.x = runningAverage.x();
	transformStamped.transform.rotation.y = runningAverage.y();
	transformStamped.transform.rotation.z = runningAverage.z();
	transformStamped.transform.rotation.w = runningAverage.w();
	
	//Update base_link
	tf2::Vector3	baseOffset;
	tf2::Transform	baseTransform;
	baseTransform.setRotation(tf2::Quaternion(runningAverage.x(), runningAverage.y(), runningAverage.z(), runningAverage.w()));
	baseTransform = baseTransform.inverse();
	baseOffset.setX(-offsetX);
	baseOffset.setY(-offsetY);
	baseOffset.setZ(-offsetZ);
	baseOffset = baseTransform * baseOffset;
	transformStamped_base.header.stamp = ros::Time::now();
	transformStamped_base.header.frame_id = "/zed_tracked_frame";
	transformStamped_base.child_frame_id = "/base_link";
	transformStamped_base.transform.translation.x = baseOffset.x();
	transformStamped_base.transform.translation.y = baseOffset.y();
	transformStamped_base.transform.translation.z = baseOffset.z();
	transformStamped_base.transform.rotation.x = runningAverage.inverse().x();
	transformStamped_base.transform.rotation.y = runningAverage.inverse().y();
	transformStamped_base.transform.rotation.z = runningAverage.inverse().z();
	transformStamped_base.transform.rotation.w = runningAverage.inverse().w();
	
	std::cout << "Calib Thread closing" << std::endl;
	
	return;
}

bool AreQuaternionsClose(Eigen::Quaterniond q1, Eigen::Quaterniond q2) {
 
	float dot = q1.dot(q2);
 
	if (dot < 0.0f) {
		return false;					
	}
 
	else {
		return true;
	}
}

Eigen::Quaterniond InverseSignQuaternion(Eigen::Quaterniond q) {
	return Eigen::Quaterniond(-q.x(), -q.y(), -q.z(), -q.w());
}

Eigen::Quaterniond AverageQuaternion(Eigen::Vector4d &cumulative, Eigen::Quaterniond newRotation, Eigen::Quaterniond firstRotation, int addAmount) {
 
	double w = 0.0f;
	double x = 0.0f;
	double y = 0.0f;
	double z = 0.0f;
 
	//Before we add the new rotation to the average (mean), we have to check whether the quaternion has to be inverted. Because
	//q and -q are the same rotation, but cannot be averaged, we have to make sure they are all the same.
	if (!AreQuaternionsClose(newRotation, firstRotation)) {
 
		newRotation = InverseSignQuaternion(newRotation);	
	}
	
	//Average the values
	double addDet = 1.0 / (double)addAmount;
	cumulative[3] += newRotation.w();
	w = cumulative[3] * addDet;
	cumulative[0] += newRotation.x();
	x = cumulative[0] * addDet;
	cumulative[1] += newRotation.y();
	y = cumulative[1] * addDet;
	cumulative[2] += newRotation.z();
	z = cumulative[2] * addDet;		
 
	//note: might need normalisation
	return Eigen::Quaterniond(w, x, y, z);
}