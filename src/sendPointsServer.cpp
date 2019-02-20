#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <test/Pointspub.h>
#include <cmath>
#define EDGE_OF_CIRCLE 18
#define EDGE_OF_RECTANGLE 4
#define PI 3.1415926


class PointsPublisherServer{

public:
    PointsPublisherServer(){
    	pub = nh.advertise<geometry_msgs::PointStamped>("/clicked_point", EDGE_OF_CIRCLE + 2);
        service = nh.advertiseService("points_publisher_server", &PointsPublisherServer::handle_function,this);
    }
    
    bool handle_function(test::Pointspub::Request &req, test::Pointspub::Response &res);
    
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::ServiceServer service;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "points_publisher_server"); 
    PointsPublisherServer pointspublisherserverobject;
    ros::spin();//用于触发topic、service的响应队列
    return 0;
}

bool PointsPublisherServer::handle_function(test::Pointspub::Request &req, test::Pointspub::Response &res){
    int edge_num=0;
    float radius=0.0, width=0.0,length=0.0;
    if(req.geometry == "circle"){
        edge_num = EDGE_OF_CIRCLE;
        radius = req.range[0];
        //publish points into /click_points
        if(ros::ok()){
            for(int i = 1;i < edge_num + 2; ++i){
                geometry_msgs::PointStamped point;
            	point.header.seq = i;
            	point.header.stamp = ros::Time::now();
            	point.header.frame_id = "odom";
            	point.point.x = req.center.x + radius*sin(i*2*PI/edge_num);
            	point.point.y = req.center.y + radius*cos(i*2*PI/edge_num);
            	point.point.z = 0;
            	pub.publish(point);
            	ros::Duration(0.5).sleep();
            }
            geometry_msgs::PointStamped middle_point;
            middle_point.header.seq = edge_num + 2;
            middle_point.header.stamp = ros::Time::now();
            middle_point.header.frame_id = "odom";
            middle_point.point.x = req.center.x;
            middle_point.point.y = req.center.y;
            middle_point.point.z = 0;
            pub.publish(middle_point);
            ROS_INFO("%i points of circle have been published", edge_num + 2);
        }
    }
    else if(req.geometry == "rectangle"){
        edge_num = EDGE_OF_RECTANGLE;
        width = req.range[0];
        length = req.range[1];
        if(ros::ok()){
	    geometry_msgs::PointStamped point;
	    //point upper right
            point.header.seq = 1;
            point.header.stamp = ros::Time::now();
            point.header.frame_id = "odom";
            point.point.x = req.center.x + length/2;
            point.point.y = req.center.y + width/2;
            point.point.z = 0;
            pub.publish(point);
            ROS_INFO("1 point published");
            ros::Duration(0.5).sleep();
            //point upper left
            point.header.seq = 2;
            point.header.stamp = ros::Time::now();
            point.point.x = req.center.x - length/2;
            point.point.y = req.center.y + width/2;
            pub.publish(point);
            ROS_INFO("2 point published");
            ros::Duration(0.5).sleep();
            //point bottom left
            point.header.seq = 3;
            point.header.stamp = ros::Time::now();
            point.point.x = req.center.x - length/2;
            point.point.y = req.center.y - width/2;
            pub.publish(point);
            ROS_INFO("3 point published");
            ros::Duration(0.5).sleep();
            //point bottom right
            point.header.seq = 4;
            point.header.stamp = ros::Time::now();
            point.point.x = req.center.x + length/2;
            point.point.y = req.center.y - width/2;
            pub.publish(point);
            ROS_INFO("4 point published");
            ros::Duration(0.5).sleep();
            //point upper right
            point.header.seq = 5;
            point.header.stamp = ros::Time::now();
            point.point.x = req.center.x + length/2;
            point.point.y = req.center.y + width/2;
            pub.publish(point);
            ROS_INFO("5 point published");
            ros::Duration(0.5).sleep();
            //middle point
            geometry_msgs::PointStamped middle_point;
            middle_point.header.seq = edge_num + 2;
            middle_point.header.stamp = ros::Time::now();
            middle_point.header.frame_id = "odom";
            middle_point.point.x = req.center.x;
            middle_point.point.y = req.center.y;
            middle_point.point.z = 0;
            pub.publish(middle_point);
            ROS_INFO("middle point published");
        }
        res.feedback="points published";
    }
    else{
    	res.feedback="The type of geometry is not matched!";
    }  

    return true;
}


/*

bool handle_function(test::Pointspub::Request &req, test::Pointspub::Response &res){
    int edge_num=0;
    float radius=0.0, width=0.0,length=0.0;
    if(req.geometry == "circle"){
        edge_num = EDGE_OF_CIRCLE;
        radius = req.range[0];
        //publish points into /click_points
        if(ros::ok()){
            for(int i = 1;i < edge_num + 2; ++i){
                geometry_msgs::PointStamped point;
            	point.header.seq = i;
            	point.header.stamp = ros::Time::now();
            	point.header.frame_id = "odom";
            	point.point.x = req.center.x + radius*sin(i*2*PI/edge_num);
            	point.point.y = req.center.y + radius*cos(i*2*PI/edge_num);
            	point.point.z = 0;
            	pub.publish(point);
            	ros::Duration(0.5).sleep();
            }
            geometry_msgs::PointStamped middle_point;
            middle_point.header.seq = edge_num + 2;
            middle_point.header.stamp = ros::Time::now();
            middle_point.header.frame_id = "odom";
            middle_point.point.x = req.center.x;
            middle_point.point.y = req.center.y;
            middle_point.point.z = 0;
            pub.publish(middle_point);
            ROS_INFO("%i points of circle have been published", edge_num + 2);
        }
    }
    else if(req.geometry == "rectangle"){
        edge_num = EDGE_OF_RECTANGLE;
        width = req.range[0];
        length = req.range[1];
        if(ros::ok()){
	    geometry_msgs::PointStamped point;
	    //point upper right
            point.header.seq = 1;
            point.header.stamp = ros::Time::now();
            point.header.frame_id = "odom";
            point.point.x = req.center.x + length/2;
            point.point.y = req.center.y + width/2;
            point.point.z = 0;
            pub.publish(point);
            ROS_INFO("1 point published");
            ros::Duration(0.5).sleep();
            //point upper left
            point.header.seq = 2;
            point.header.stamp = ros::Time::now();
            point.point.y = req.center.y - width/2;
            pub.publish(point);
            ROS_INFO("2 point published");
            ros::Duration(0.5).sleep();
            //point bottom left
            point.header.seq = 3;
            point.header.stamp = ros::Time::now();
            point.point.x = req.center.x - length/2;
            pub.publish(point);
            ROS_INFO("3 point published");
            ros::Duration(0.5).sleep();
            //point bottom right
            point.header.seq = 4;
            point.header.stamp = ros::Time::now();
            point.point.x = req.center.x + length/2;
            pub.publish(point);
            ROS_INFO("4 point published");
            ros::Duration(0.5).sleep();
            //point upper right
            point.header.seq = 5;
            point.header.stamp = ros::Time::now();
            point.point.x = req.center.y + width/2;
            pub.publish(point);
            ROS_INFO("5 point published");
            ros::Duration(0.5).sleep();
            //middle point
            geometry_msgs::PointStamped middle_point;
            middle_point.header.seq = edge_num + 2;
            middle_point.header.stamp = ros::Time::now();
            middle_point.header.frame_id = "odom";
            middle_point.point.x = req.center.x;
            middle_point.point.y = req.center.y;
            middle_point.point.z = 0;
            pub.publish(middle_point);
            ROS_INFO("middle point published");
        }
        res.feedback="points published";
    }
    else{
    	res.feedback="The type of geometry is not matched!";
    }  

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "points_publisher_server"); 
    ros::NodeHandle nh;
    //....节点功能
    //....
    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("/click_points", EDGE_OF_CIRCLE + 1);
     ros::ServiceServer service = nh.advertiseService("points_publisher_server", handle_function);
    ros::spin();//用于触发topic、service的响应队列
    return 0;
}

*/







