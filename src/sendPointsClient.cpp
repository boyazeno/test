#include <ros/ros.h>
#include <test/Pointspub.h>
#include <string.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_publisher_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<test::Pointspub>("points_publisher_server");
    std::string geometry;
    float cx,cy,cz, range_1,range_2;
    
    nh.param<std::string>("geometry",geometry,"circle");
    nh.param<float>("cx",cx, 0.0);
    nh.param<float>("cy",cy, 0.0);
    nh.param<float>("cz",cz, 0.0);
    nh.param<float>("range_1",range_1,3.5);
    nh.param<float>("range_2",range_2,0.0);
    // 实例化srv，设置其request消息的内容，这里request包含两个变量，name和age，见Greeting.srv
    test::Pointspub srv;
    srv.request.geometry = geometry;
    srv.request.center.x = cx;
    srv.request.center.y = cy;
    srv.request.center.z = cz;
    srv.request.range.resize(2);
    srv.request.range[0] = range_1;
    srv.request.range[1] = range_2;

    if (client.call(srv))
    {
        ROS_INFO("Response from server: %s", srv.response.feedback.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service Service points_publisher");
        return 1;
    }
    return 0;
}
