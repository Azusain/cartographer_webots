#include "string.h"
#include "cartographer_webots/Goalname.h" 
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
using namespace std;
 
ros::NodeHandle *n;
ros::Publisher pub_goal;          

void goalCallback(const cartographer_webots::Goalname::ConstPtr &value){
    int isture=0;
    string goal_name = value->goal_name;
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.seq = 1;
    target_pose.header.frame_id = "map";
    if (isture){
        target_pose.header.stamp = ros::Time::now();
        pub_goal.publish(target_pose);
        ROS_INFO("Ready to go to the goal %s",goal_name.c_str());
    }
    else{
        ROS_ERROR("Can't compare the goal");
    }
}

// This node get the 'set_goal' command from rviz and then send it to the internal navigation node,
// then the navigation node generates the velocity data and send it to the 'demo_2nav_move' node.
int main(int argc, char **argv) {
    // create a node named 'robot' on ROS network
    ros::init(argc, argv, "robot_set_goal");
    n = new ros::NodeHandle;
    ros::Subscriber sub_goal;
    sub_goal = n->subscribe("/robot/goal",1,goalCallback);
    
    pub_goal = n->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",2);
    ROS_INFO("Started success ");
    ros::spin();
}
