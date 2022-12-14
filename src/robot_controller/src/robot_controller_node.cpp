#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"

enum
{
    KEYBOARD,
    JOYCON
} mode;
bool useTri = false;

void keyboardCB(const geometry_msgs::Twist::ConstPtr &msg);
void joyConCB(const sensor_msgs::Joy::ConstPtr &msg);

ros::Publisher speedPub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_controller_node");
    ros::NodeHandle n;
    mode = KEYBOARD;

    speedPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 20);

    ros::Subscriber keySub = n.subscribe("/keyboard/cmd_vel", 20, keyboardCB);
    ros::Subscriber joySub = n.subscribe("/joy", 20, joyConCB);

    ros::Rate loop_rate(1000);
    int count = 0;

    ROS_INFO_STREAM("Robot Controller Node STARTED.");
    ros::spin();
    return 0;
}

void keyboardCB(const geometry_msgs::Twist::ConstPtr &msg)
{
    if (mode == KEYBOARD)
    {
        static geometry_msgs::Twist speed;
        speed.angular = msg.get()->angular;
        speed.linear = msg.get()->linear;
        speedPub.publish(speed);
    }
    else
    {
        ROS_WARN_STREAM("YOU ARE IN JOY CON MODE");
    }
}

static float lastX = 0;
static float lastO = 0;
void joyConCB(const sensor_msgs::Joy::ConstPtr &msg)
{
    if (msg.get()->buttons[0] == 1)
    {
        if (mode == JOYCON)
        {
            mode = KEYBOARD;
            ROS_INFO_STREAM("Switch to KEYBOARD Mode");
        }
        else
        {
            mode = JOYCON;
            ROS_INFO_STREAM("Switch to JON CON Mode");
        }
    }

    if (mode != JOYCON)
    {
        ROS_WARN_STREAM("YOU ARE IN KEYBOARD MODE");
        return ;
    }

    if (msg.get()->buttons[3] == 1)
    {
        useTri = !useTri;
        if (useTri) ROS_INFO_STREAM("Switch to Trigger Control");
        else ROS_INFO_STREAM("Switch to Joy Control");
    }

    static geometry_msgs::Twist speed;
    float ang = 0;
    float lin = 0;
    if (useTri) {
        ang = abs(msg.get()->axes[3]) < 0.05 ? 0 : msg.get()->axes[3]*2;
        lin = (msg.get()->axes[2] - msg.get()->axes[5]) * 0.7;
    }
    else {
        ang = abs(msg.get()->axes[0]) < 0.05 ? 0 : msg.get()->axes[0]*2;
        lin = abs(msg.get()->axes[3]) < 0.05 ? 0 : msg.get()->axes[3]*1.5;
    }
    if ((ang != 0 || ang == 0 && lastO == 0) && (lin != 0|| lin == 0 && lastX == 0) && abs(ang - lastO) <= 0.1 && abs(lin - lastX) <= 0.2)
        return ;
    speed.angular.z = lastO;
    speed.linear.x = lastX;
    if (ang == 0 && lastO != 0)
        lastO = speed.angular.z = 0;
    if (ang != 0 && abs(ang - lastO) > 0.1)
        lastO = speed.angular.z = ang;
    if (lin == 0 && lastX != 0)
        lastX = speed.linear.x = 0;
    if (lin != 0 && abs(lin - lastX) > 0.1)
        lastX = speed.linear.x = lin;
    speedPub.publish(speed);
}