#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "obstacle_detector/Obstacles.h"

void callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
	ROS_INFO("data: [%s]", obstacle_msg->circles[0].radius);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "static_controller");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/raw_obstacles", 100, callback);

	ros::spin();

	return 0;
}

