#include <ros/ros.h>
#include <test/Pointspub.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_publisher_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<test::Pointspub>("points_publisher_server");


    // 实例化srv，设置其request消息的内容，这里request包含两个变量，name和age，见Greeting.srv
    test::Pointspub srv;
    srv.request.geometry = "rectangle";
    srv.request.center.x = 0.0;
    srv.request.center.y = 0.0;
    srv.request.center.z = 0.0;
    srv.request.range.resize(2);
    srv.request.range[0] = 10.0;
    srv.request.range[1] = 15.0;

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
