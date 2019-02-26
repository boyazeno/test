#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>


#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>

#include <frontier_exploration/geometry_tools.h>

#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/ExploreTaskActionGoal.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <costmap_2d/footprint.h>

#include <ros/wall_timer.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <cmath>
#define EDGE_OF_CIRCLE 18
#define EDGE_OF_RECTANGLE 4
#define PI 3.1415926


int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_client");
    ros::NodeHandle nh_;
    geometry_msgs::PolygonStamped input_;
    geometry_msgs::PointStamped center_point;
    
    center_point.header.stamp = ros::Time::now();
    center_point.header.frame_id = "base_link";
    center_point.point.x = center_point.point.y = center_point.point.z = 0.0;
    
    input_.header.stamp = ros::Time::now();
    input_.header.frame_id = "base_link";
    input_.polygon.point.resize(18);
    for(int i=0; i<18;++i){
        geometry_msgs::Point32 point;
        point.point.x = 4.0*sin(i*2*PI/edge_num);
        point.point.y = 4.0**cos(i*2*PI/edge_num);
        point.point.z = 0;
        pub.publish(point);
        input_.polygon.point[i]=point;
    } 
    
    actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> exploreClient("explore_server", true);
    exploreClient.waitForServer();
    ROS_INFO("Sending goal");
    frontier_exploration::ExploreTaskGoal goal;
    goal.explore_center = center_point;
    goal.explore_boundary = input_;
    exploreClient.sendGoal(goal); 

    bool finished_before_timeout = exploreClient.waitForResult(ros::Duration(30.0));
                
    if (finished_before_timeout){
        actionlib::SimpleClientGoalState state = exploreClient.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
     ROS_INFO("Action did not finish before the time out.");
                
    ros::spin();
    return 0;
}


