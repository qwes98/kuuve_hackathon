/**
 *  Ros code for path planning and calculate steering value from obstacle data
 *
 */

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "obstacle_detector/Obstacles.h"
#include "obstacle_detector/obstacle_extractor.h"
#include "std_msgs/Header.h"

/**
 *  Node that make path and calculate steering value
 *
 *  @author: Jiwon Park
 *  @date: 02.21.2018
 *
 *  subscribe: obstacles (from obstacle detector node)
 *  publish: filtered obstacles
 *
 */
class StaticControlNode 
{
public:
	StaticControlNode();
	void callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg);

private:
	/**
	 *  Method that extracts circles(obstacles) we want and publishes them
	 *	
	 */
	void publishObstacles(const std_msgs::Header& header, const std::list<obstacle_detector::CircleObstacle>& circles);

	// TODO: Implement path planning and control algorithm and Publish steering value - Jiwon Park 02/21/2018

	ros::NodeHandle nh_;
	ros::Publisher filtered_circle_pub_;
	ros::Subscriber obstacle_sub_;
};

// ----------------------------------------------------------------

StaticControlNode::StaticControlNode()
{
	obstacle_sub_ = nh_.subscribe("/raw_obstacles", 100, &StaticControlNode::callback, this);
	filtered_circle_pub_ = nh_.advertise<obstacle_detector::Obstacles>("/filtered_circles", 100);
}

void StaticControlNode::callback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
	std::list<obstacle_detector::CircleObstacle> circles;
	int circle_count = obstacle_msg->circles.size();

	// Select obstacles only that we want to avoid
	for(int i = 0; i < circle_count; i++) {
		double center_y = obstacle_msg->circles[i].center.y;
		if(center_y < 1 && center_y > -1) {		// Extract circles
			double center_x = obstacle_msg->circles[i].center.x;
			ROS_INFO("Obstacle location: [%f, %f]", center_x, center_y);
			circles.push_back(obstacle_msg->circles[i]);	// Store circles in a list
		}
	}

	ROS_INFO("-----------------------------------------");
	publishObstacles(obstacle_msg->header, circles);
}

void StaticControlNode::publishObstacles(const std_msgs::Header& header, const std::list<obstacle_detector::CircleObstacle>& circles) {
  obstacle_detector::ObstaclesPtr obstacles_msg(new obstacle_detector::Obstacles);
  obstacles_msg->header.stamp = header.stamp;

  /*
  if (p_transform_coordinates_) {
      tf::StampedTransform transform;
  
      try {
	        tf_listener_.waitForTransform(p_frame_id_, base_frame_id_, stamp_, ros::Duration(0.1));
	        tf_listener_.lookupTransform(p_frame_id_, base_frame_id_, stamp_, transform);
	      
      catch (tf::TransformException& ex) {
	        ROS_INFO_STREAM(ex.what());
	        return;
	      }
  
      tf::Vector3 origin = transform.getOrigin();
      double theta = tf::getYaw(transform.getRotation());
  
      for (Segment& s : segments_) {
	        s.first_point = transformPoint(s.first_point, origin.x(), origin.y(), theta);
	        s.last_point = transformPoint(s.last_point, origin.x(), origin.y(), theta);
	      }
  
      for (Circle& c : circles_)
        c.center = transformPoint(c.center, origin.x(), origin.y(), theta);
  
      obstacles_msg->header.frame_id = p_frame_id_;
    }
  else
  */
    obstacles_msg->header.frame_id = header.frame_id;


  /* 
  for (const Segment& s : segments_) {
      SegmentObstacle segment;
  
      segment.first_point.x = s.first_point.x;
      segment.first_point.y = s.first_point.y;
      segment.last_point.x = s.last_point.x;
      segment.last_point.y = s.last_point.y;
  
      obstacles_msg->segments.push_back(segment);
    }
	*/

  for (const obstacle_detector::CircleObstacle& c : circles) {
		  obstacles_msg->circles.push_back(c);
  }

  filtered_circle_pub_.publish(obstacles_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "static_controller");
	StaticControlNode static_control_node;

	ros::spin();
}

