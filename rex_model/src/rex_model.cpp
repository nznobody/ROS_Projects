
#include <iostream>
#include <ostream>
#include <sstream>
#include <fstream>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Publisher pubPose;

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
	ros::shutdown();
}

struct JointDescription
{
	std::string name;
	std::string parent;
	std::string child;
	int	parentID;	//This is the location in the vector for quick looking up
	int	childID;
	static ros::Publisher pubJoints;
	double angle = 0.0;
	
	void calcAngle(std::vector<double> &angles)
	{
		if (angles.size() <= parentID || angles.size() <= childID) //Range checking
			angle = 0;	//Or keep old value?
		else 
			angle = (angles[childID] - angles[parentID]);
	}
	static void publish(std::vector<JointDescription> &joints)
	{
		sensor_msgs::JointStatePtr	output(new sensor_msgs::JointState());
		output->header.frame_id = "base_link"; //maybe this should be pelvis?
		output->header.stamp = ros::Time::now();
		for (JointDescription var : joints)
		{
			output->name.push_back(var.name);
			output->position.push_back(var.angle);
		}
		if (JointDescription::pubJoints.getNumSubscribers())
			JointDescription::pubJoints.publish(output);
	}
	static void publish(std::vector<JointDescription> &joints, std::vector<double> &angles)
	{
		sensor_msgs::JointStatePtr	output(new sensor_msgs::JointState());
		output->header.frame_id = "base_link"; //maybe this should be pelvis?
		output->header.stamp = ros::Time::now();
		for (JointDescription var : joints)
		{
			var.calcAngle(angles);
			output->name.push_back(var.name);
			output->position.push_back(var.angle);
		}
		if (JointDescription::pubJoints.getNumSubscribers())
			JointDescription::pubJoints.publish(output);
	}
};

ros::Publisher JointDescription::pubJoints;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "rexmodel", ros::init_options::NoSigintHandler);

	//std::cout << "Hello Manu" << std::endl;
	
	//For easy debugging these have been filled in.
	std::string jointNames = "pelvis_to_right_upperleg,pelvis_to_left_upperleg,right_upperleg_to_right_lowerleg,left_upperleg_to_left_lowerleg,right_lowerleg_to_right_foot,left_lowerleg_to_left_foot";	//Defaults to empty list (CSV)
	std::string angleNames = "pelvis,left_upperleg,right_upperleg,left_lowerleg,right_lowerleg,left_foot,right_foot";	//Defaults to empty list (CSV)
	std::string anglesFileName = "vrex_trace_angles.csv";
	
	ros::NodeHandle n;
	ros::NodeHandle nh_ns("~");	//Local Namespace NodeHandle
	nh_ns.getParam("joints", jointNames);
	nh_ns.getParam("angles", angleNames);
	nh_ns.getParam("anglesFileName", anglesFileName);
	
	//split the input joinst string (CSV)
	std::stringstream ssJ(jointNames);
	std::string temp;
	std::vector<std::string> joint_names;
	while (std::getline(ssJ, temp, ',')) {
		joint_names.push_back(temp);
	}
	
	//split the input joinst string (CSV)
	std::stringstream ssA(angleNames);
	std::vector<std::string> angle_names;
	while (std::getline(ssA, temp, ',')) {
		angle_names.push_back(temp);
	}
	
	std::vector<JointDescription> joints;
	//decode joint names to related angles: find the _to_, find angleIDs, populate struct
	for (std::string i_joint : joint_names)
	{
		std::string from_angle = i_joint.substr(0, i_joint.find("_to_"));
		std::string to_angle = i_joint.substr(from_angle.length() + 4, std::string::npos);
		//Try to find angles in angle list
		int parentID = -1, childID = -1, counter = 0;
		for (std::string i_angle : angle_names)
		{
			if (i_angle == from_angle)
				parentID = counter;
			if (i_angle == to_angle)
				childID = counter;
			counter++;
		}
		if (!from_angle.empty() && !to_angle.empty() && parentID != -1 && childID != -1)
		{
			JointDescription temp;
			temp.name = i_joint;
			temp.parent = from_angle;
			temp.child = to_angle;
			temp.childID = childID;
			temp.parentID = parentID;
			joints.push_back(temp);
		}

	} 

	//Read angles from file if passed
	if (anglesFileName == "")
		return -1;	//Throw ROS_ERROR maybe?
	
	std::vector<std::vector<double>> csvData;
	std::ifstream infile(anglesFileName.c_str(), std::ifstream::in);

	while (infile)
	{
		std::string s;
		if (!getline(infile, s)) break;
		std::istringstream ss(s);
		std::vector<double> record;
		while (ss)
		{
			std::string s;
			if (!getline(ss, s, ',')) break;
			record.push_back(std::atof(s.c_str()));
		}
		csvData.push_back(record);
	}
	if (!infile.eof())
	{
		//std::cerr << "Fooey!\n";
	}
	
	//Set up publishers
	JointDescription::pubJoints = nh_ns.advertise<sensor_msgs::JointState>("joints", 10);
	//pubPose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 100);
	
	//Test it
	
	signal(SIGINT, mySigintHandler);
	
	ros::Rate	loopRate(5);
	
	int step = 0;
	while (ros::ok())
	{
		if (step >= csvData.size())
			step = 0;
		if (!csvData[step].size())
		{
			step++;
			continue;
		}
		JointDescription::publish(joints, csvData[step]);
		ros::spinOnce();
		loopRate.sleep();
		step++;
	}
	
	ros::spin();
	
	ROS_WARN("Shutdown request received by rexmodel");
	pubPose.shutdown();
	ros::shutdown();
	return 0;
}