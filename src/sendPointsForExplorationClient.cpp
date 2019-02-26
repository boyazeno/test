#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <frontier_exploration/geometry_tools.h>

#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/ExploreTaskActionGoal.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/ExploreRange.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <costmap_2d/footprint.h>
#include <tf/transform_listener.h>

#include <ros/wall_timer.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <visualization_msgs/Marker.h>
#include <boost/foreach.hpp>
#define SMALL_RANGE 4.5
#define MULTIPLIER 1.5


namespace frontier_exploration{

/**
 * @brief Client for FrontierExplorationServer that receives control points from rviz, and creates boundary polygon for frontier exploration
 */
class FrontierExplorationClient{

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::ServiceServer service_;
    ros::Publisher point_viz_pub_;
    ros::WallTimer point_viz_timer_;
    geometry_msgs::PolygonStamped input_;

    bool waiting_for_center_;

    /**
     * @brief Publish markers for visualization of points for boundary polygon.
     */
    void vizPubCb(){

        visualization_msgs::Marker points, line_strip;

        points.header = line_strip.header = input_.header;
        points.ns = line_strip.ns = "explore_points";

        points.id = 0;
        line_strip.id = 1;

        points.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        if(!input_.polygon.points.empty()){

            points.action = line_strip.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

            points.scale.x = points.scale.y = 0.1;
            line_strip.scale.x = 0.05;

            BOOST_FOREACH(geometry_msgs::Point32 point, input_.polygon.points){
                line_strip.points.push_back(costmap_2d::toPoint(point));
                points.points.push_back(costmap_2d::toPoint(point));
            }

            if(waiting_for_center_){
                line_strip.points.push_back(costmap_2d::toPoint(input_.polygon.points.front()));
                points.color.a = points.color.r = line_strip.color.r = line_strip.color.a = 1.0;
            }else{
                points.color.a = points.color.b = line_strip.color.b = line_strip.color.a = 1.0;
            }
        }else{
            points.action = line_strip.action = visualization_msgs::Marker::DELETE;
        }
        point_viz_pub_.publish(points);
        point_viz_pub_.publish(line_strip);

    }

    /**
     * @brief call back function of send_Points_For_Exploration_Server
     * @param point Req Res request and response from service
     */
    bool pointCb(frontier_exploration::ExploreRange::Request &req, frontier_exploration::ExploreRange::Response &res){
      
       // double average_distance = polygonPerimeter(input_.polygon) / input_.polygon.points.size();
        //extracte information from request into range, middle point; response is string of feedback.
        double range = req->range;
        geometry_msgs::Point mpoint = req->center;
        double local_range = SMALL_RANGE;
        geometry_msgs::PointStamped point;
        geometry_msg::PolygonStamped input_;
        
        actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> exploreClient("explore_server", true);
        exploreClient.waitForServer();

        /*the local detect range is not bigger than 2*gloable range*/        
        while(local_range < 2*range){
                
        //do exploration for one time   
        ROS_INFO("Sending goal");
        frontier_exploration::ExploreTaskGoal goal;
        //define the center point of local polygon
        point.header.frame_id = "base_link";
        point.header. stamp = ros::Time::now();
        point.point.x = point.point.y = point.point.z=0.0;
        //generate local polygon, and stored in input_
        input_.header.frame_id = "base_link";
        input_.header.stamp = ros::Time::now();
        input_.polygon.points.resize(18);
        for(int i=0; i<18; ++i){
            geometry_msgs::Point32 temp;
            temp.point.x = local_range*cos(i*2*PI/18);
            temp.point.y = local_range*sin(i*2*PI/18);
            temp.point.z = 0.0;
            input_.polygon.points[i]= temp;
        }
        goal.explore_center = *point;
        goal.explore_boundary = input_;
        exploreClient.sendGoal(goal);  //here can use a callback function to get the current location.
        
        //check whether the action is finished or time out.           
        bool finished_before_timeout = exploreClient.waitForResult(ros::Duration(20.0));
   
        if (exploreClient.getState()==actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
        {
            //the action is successfully finished, so the local detect range will be larger
            local_range = local_range * MULTIPLIER;
            
            continue;
        }
        else
        {
            //action is failed or timeout, which means the local region is not fully detected. should send the same detect range (base_link) again to detect another place with the same range.
            local_range = SMALL_RANGE;
            ROS_INFO("Action did not finish before the time out or action %s",state.toString().c_str()");
            
        }
        
        }
        
        
    }

public:

    /**
     * @brief Constructor for the client.
     */
    FrontierExplorationClient() :
        nh_(),
        private_nh_("~"),
        waiting_for_center_(false)
    {
        input_.header.frame_id = "map";
        
        service_ = nh.advertiseService("send_Points_For_Exploration_Server", &FrontierExplorationClient::pointCb, this);
        point_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("exploration_polygon_marker", 10);
        point_viz_timer_ = nh_.createWallTimer(ros::WallDuration(0.05), boost::bind(&FrontierExplorationClient::vizPubCb, this));
        ROS_INFO("Please use the 'Point' tool in Rviz to select an exporation boundary.");
    }    

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_client");

    frontier_exploration::FrontierExplorationClient client;
    ros::spin();
    return 0;
}
