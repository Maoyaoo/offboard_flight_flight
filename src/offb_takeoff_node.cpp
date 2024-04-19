/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

//该节点控制飞机起飞

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

//我们创建了一个简单的回调函数来储存飞控当前的状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    //我们构建了一个发布者来发布本地位置指令并请求客户端进行加解锁状态及控制模式的切换
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    //在发布任何消息之前，我们需要等待飞控和MAVROS建立连接。 在收到心跳包之后，代码便会跳出这个循环。
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //尽管PX4在航空航天常用的NED坐标系下操控飞机，但MAVROS将自动将该坐标系切换至常规的ENU坐标系下，反之亦然。 
    //这也就是为什么我们设置z为+2。
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // send a few setpoints before starting
    //在进入off - board模式之前，您必须已经开始流式传输设定值。
    //否则，模式切换将被拒绝。 这里的100 可以被设置为任意数。
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //我们将自定义模式设置为 OFFBOARD
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        //首先判断当前模式是否为offboard模式，如果不是，则客户端set_mode_client向服务端offb_set_mode发起请求call，
        //然后服务端回应response将模式返回，这就打开了offboard模式
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            //else指已经为offboard模式，然后进去判断是否解锁，如果没有解锁，则客户端arming_client向服务端arm_cmd发起请求call
            //然后服务端回应response成功解锁，这就解锁了
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose); //发布位置信息

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
