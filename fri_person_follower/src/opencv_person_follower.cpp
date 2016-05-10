#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <opencv_person_detector/HumanArea.h>
#include <sound_play/SoundRequest.h>
#include <string>

using opencv_person_detector::HumanArea;
using opencv_person_detector::Human;

ros::Publisher sound_pub;
sound_play::SoundRequest sound;
move_base_msgs::MoveBaseGoal goal;
bool received_cb = false;
double x = 0.0;
double yaw = 0.0;
std::string last_move = "";

/**
 * Callback for the HumanArea from the OpenCV person detector
 * Determines which direction to turn or to go forward and sets the goal
 */
void human_cb(const HumanArea::ConstPtr& input) {
    received_cb = true;
    Human human = input->human;
    geometry_msgs::Point img_center = input->image_center;

     // Determining movement
    if (human.human_center.x + 50 < img_center.x) {
        // Turn to the left
        if (img_center.x - human.human_center.x > 100) {
            yaw = 0.21;
        } else {
            yaw = 0.205;
        }

        // Checking if needs to announce movement
        if (last_move != "left") {
            last_move = "left";
            sound.arg = "left";
            sound_pub.publish(sound);
        }
    } else if (human.human_center.x - 50 > img_center.x) {
        // Turn to the right
        if (human.human_center.x - img_center.x > 100) {
            yaw = -0.21;
        } else {
            yaw = -0.205;
        }

        // Checking if needs to announce movement
        if (last_move != "right") {
            last_move = "right";
            sound.arg = "right";
            sound_pub.publish(sound);
        }
    } else {
        // Move forward 
        // Uses heuristic that if the height of the person is less than
        // 390 then robot still needs to move forward
        if (human.height < 400) {
            if (human.height < 370) {
                x = 0.33;
            } else {
                x = 0.28;
            }

            // Checking if needs to announce movement
            if (last_move != "forward") {
                last_move = "forward";
                sound.arg = "forward";
                sound_pub.publish(sound);
            }
        }
    }

    // Setting movement in goal
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    goal.target_pose.pose.position.x = x;
}

int main(int argc, char** argv) {
    // Initializing ros
    ros::init(argc, argv, "Opencv_Person_Follower");
    ros::NodeHandle n;

    // Initializing objects
    sound_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 1);
    ros::Subscriber sub = n.subscribe("/opencv_person_detector/center_points", 1, human_cb);
    ros::Rate loop_rate(5);
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    goal.target_pose.header.frame_id = "/base_link";
    sound.command = 1;
    sound.sound = -3;

    // Waiting for first callback
    while (ros::ok() && !received_cb) {
        ros::spinOnce();
    }

    // Main loop: detemining goal with callback and publishing that goal
    while (ros::ok()) {
        ros::spinOnce();

        // Checking that the robot needs to move
        if (yaw != 0 ||  x != 0) {
            ac.sendGoal(goal);
            ac.waitForResult();
        } else if (last_move != "done") {
            sound.arg = "done";
            last_move = "done";
            sound_pub.publish(sound);
        }

        // Resetting x and yaw
        x = 0.0;
        yaw = 0.0;
        loop_rate.sleep();
    }

    return 0;
}
