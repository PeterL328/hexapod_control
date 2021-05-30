#include <ros/ros.h>
#include <ros/console.h>

#include <hexapod_msgs/LegsJoints.h>

int main(int argc, char **argv)
{
    const std::string node_name = "test_joints";
    const std::string joints_command_topic_name = "joints_command";

    ros::init(argc, argv, node_name);
    ros::NodeHandle n("~");

    // Get values from arguments
    float publish_rate_in_hz;
    n.param("publish_rate", publish_rate_in_hz, 0.3f);
    ROS_INFO("Publishing at %fHz on %s.", publish_rate_in_hz, joints_command_topic_name.c_str());

    ros::Publisher test_joints_pub = n.advertise<hexapod_msgs::LegsJoints>(joints_command_topic_name, 1);
    ros::Rate loop_rate(publish_rate_in_hz);

    hexapod_msgs::LegsJoints msg1;
    msg1.left_front_leg.coxa = 0.f;
    msg1.left_front_leg.femur = 0.f;
    msg1.left_front_leg.tibia = 0.f;

    msg1.left_mid_leg.coxa = 0.f;
    msg1.left_mid_leg.femur = 0.f;
    msg1.left_mid_leg.tibia = 0.f;

    msg1.left_back_leg.coxa = 0.f;
    msg1.left_back_leg.femur = 0.f;
    msg1.left_back_leg.tibia = 0.f;

    msg1.right_front_leg.coxa = 0.f;
    msg1.right_front_leg.femur = 0.f;
    msg1.right_front_leg.tibia = 0.f;

    msg1.right_mid_leg.coxa = 0.f;
    msg1.right_mid_leg.femur = 0.f;
    msg1.right_mid_leg.tibia = 0.f;

    msg1.right_back_leg.coxa = 0.f;
    msg1.right_back_leg.femur = 0.f;
    msg1.right_back_leg.tibia = 0.f;

    hexapod_msgs::LegsJoints msg2;
    msg2.left_front_leg.coxa = 0.f;
    msg2.left_front_leg.femur = 0.f;
    msg2.left_front_leg.tibia = 0.f;

    msg2.left_mid_leg.coxa = 0.f;
    msg2.left_mid_leg.femur = -0.349066f;
    msg2.left_mid_leg.tibia = 0.f;

    msg2.left_back_leg.coxa = 0.f;
    msg2.left_back_leg.femur = 0.f;
    msg2.left_back_leg.tibia = 0.f;

    msg2.right_front_leg.coxa = 0.f;
    msg2.right_front_leg.femur = -0.349066f;
    msg2.right_front_leg.tibia = 0.f;

    msg2.right_mid_leg.coxa = 0.f;
    msg2.right_mid_leg.femur = 0.f;
    msg2.right_mid_leg.tibia = 0.f;

    msg2.right_back_leg.coxa = 0.f;
    msg2.right_back_leg.femur = -0.349066f;
    msg2.right_back_leg.tibia = 0.f;

    int i = 0;
    while (ros::ok())
    {
        if (i % 2 == 0) {
            test_joints_pub.publish(msg1);
        } else {
            test_joints_pub.publish(msg2);
        }
        i++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
