#include <signal.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>

#include <webots_ros.h>

ros::NodeHandle *n;

const int TIME_STEP = 32;                 
const int NMOTORS = 2;                      
const float MAX_SPEED = 2.0;                
const std::string ROBOT_NAME = "robot/";    

static int controllerCount;
static std::vector<std::string> controllerList; 

ros::Publisher odompub;                  

double GPSvalues[4];                      
int gps_flag=1;                         
double Inertialvalues[4];                 
double liner_speed=0,angular_speed=0;      

Webots w = Webots(TIME_STEP,ROBOT_NAME);

void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    controllerList.push_back(name->data);
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

void quit(int sig) {
    w.Quit(n);
}

void broadcastTransform(){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(GPSvalues[0]-GPSvalues[2],GPSvalues[1]-GPSvalues[3],0));
    tf::Quaternion q(Inertialvalues[0],Inertialvalues[2],Inertialvalues[1],-Inertialvalues[3]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","base_link"));
    transform.setIdentity();
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "robot/Sick_LMS_291"));
}

void send_odom_data()
{
    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = GPSvalues[0]-GPSvalues[2];
    odom.pose.pose.position.y = GPSvalues[1]-GPSvalues[3];
    odom.pose.pose.position.z = 0;

    odom.pose.pose.orientation.x = Inertialvalues[0];
    odom.pose.pose.orientation.y = Inertialvalues[2];
    odom.pose.pose.orientation.z = Inertialvalues[1];
    odom.pose.pose.orientation.w = -Inertialvalues[3];

    odom.twist.twist.linear.x = liner_speed;
    odom.twist.twist.angular.z = angular_speed;

    odompub.publish(odom);
}

void gpsCallback(const geometry_msgs::PointStamped::ConstPtr &value){
    GPSvalues[0] = value->point.x;
    GPSvalues[1] = value->point.z;
    if (gps_flag){
        GPSvalues[2] = value->point.x;
        GPSvalues[3] = value->point.z;
        gps_flag=0;
    }
    broadcastTransform();  
}

void inertial_unitCallback(const sensor_msgs::Imu::ConstPtr &values){
    Inertialvalues[0] = values->orientation.x;
    Inertialvalues[1] = values->orientation.y;
    Inertialvalues[2] = values->orientation.z;
    Inertialvalues[3] = values->orientation.w;
    broadcastTransform();
}

void velCallback(const nav_msgs::Odometry::ConstPtr &value){
    liner_speed = value->twist.twist.linear.x;
    angular_speed = value->twist.twist.angular.z;
    send_odom_data();
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "zh_CN.utf8"); 
    std::string controllerName;
    ros::init(argc, argv, "robot_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    // 截取退出信号
    signal(SIGINT, quit);

    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    w.Init(n, nameSub, controllerCount, controllerList);

    if(w.EnableService(n, "Sick_LMS_291")) return 1;

    ros::Subscriber gps_sub;
    if(!w.EnableService(n, "gps")){
        gps_sub = n->subscribe(std::string(ROBOT_NAME)+std::string("gps/values"), 1, gpsCallback);
        ROS_INFO("Topic for gps initialized.");
        while (gps_sub.getNumPublishers() == 0) {}
        ROS_INFO("Topic for gps connected.");
    }else return 1;
    
    ros::Subscriber inertial_unit_sub;
    if(!w.EnableService(n, "inertial_unit")){
        inertial_unit_sub = n->subscribe(std::string(ROBOT_NAME)+std::string("inertial_unit/quaternion"), 1, inertial_unitCallback);
        ROS_INFO("Topic for inertial_unit initialized.");
        while (inertial_unit_sub.getNumPublishers() == 0) {}
        ROS_INFO("Topic for inertial_unit connected.");
    }else return 1;

    ros::Subscriber sub_speed;
    sub_speed = n->subscribe("/vel", 1, velCallback);
    odompub = n->advertise<nav_msgs::Odometry>("robot/odom",10);

    while (ros::ok()) {
        if (w.ChecktimeStep())break;
        ros::spinOnce();
    }
    
    w.Quit(n); 
    return 0;
}

