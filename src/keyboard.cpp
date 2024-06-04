
#include <signal.h>
#include <locale.h> 
#include <webots_ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle *n;

const int TIME_STEP = 32;                 
const int NMOTORS = 2;                     
const float MAX_SPEED = 2.0;                
const std::string ROBOT_NAME = "robot/";   
double speeds[NMOTORS]={0.0,0.0};          
float linear_temp=0, angular_temp=0;        

static const char *motorNames[NMOTORS] ={
    "left_motor", 
    "right_motor"
};

static int controllerCount;
static std::vector<std::string> controllerList; 

ros::Publisher pub_speed;                   
Webots w = Webots(TIME_STEP,ROBOT_NAME);

void updateSpeed() {   
    nav_msgs::Odometry speed_data;
    float L = 0.6;
    speeds[0]  = 10.0*(2.0*linear_temp - L*angular_temp)/2.0;
    speeds[1]  = 10.0*(2.0*linear_temp + L*angular_temp)/2.0;
    for (int i = 0; i < NMOTORS; ++i) {
        w.SetMotorsVelocity(n, motorNames[i], -speeds[i]);
    }
    speed_data.header.stamp = ros::Time::now();
    speed_data.twist.twist.linear.x = linear_temp;
    speed_data.twist.twist.angular.z = angular_temp;
    pub_speed.publish(speed_data);
    speeds[0]=0;
    speeds[1]=0;
}

// list all the available controllers, (however we obly use keyboard as our controller).
void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    controllerList.push_back(name->data);
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

// these data is to be sent to the cartographer node.
void keyboardDataCallback(const webots_ros::Int32Stamped::ConstPtr &value) {
    switch (value->data){
        // LEFT.
        case 314:
            angular_temp-=0.1;
            break;
        // FORWARD.
        case 315:
            linear_temp += 0.1;
            break;
        // RIGHT.
        case 316:
            angular_temp+=0.1;
            break;
        // BACKWARD.
        case 317:
            linear_temp-=0.1;
            break;
        // STOP.
        case 32:
            linear_temp = 0;
            angular_temp = 0;
            break;
        default:
            break;
    }
}

void cmdvelDataCallback(const geometry_msgs::Twist::ConstPtr &value) {
    angular_temp = value->angular.z ;
    linear_temp = value->linear.x ;
}

void quit(int sig) {
    w.Quit(n);
}

// Get messages from webots controller or navigation node.
// Then send data to cartographer broadcaster node.
int main(int argc, char **argv) {
    setlocale(LC_CTYPE,"zh_CN.utf8");
    std::string controllerName;
    ros::init(argc, argv, "robot_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    signal(SIGINT, quit);

    ros::Subscriber nameSub = n->subscribe("model_name", 10, controllerNameCallback);
    w.Init(n, nameSub, controllerCount, controllerList);
    w.InitMotors(n, motorNames, NMOTORS);
    
    ros::Subscriber cmdvelSub;
    // It should be noted that:
    // we will get velocity data from the /cmd_vel topic, if navigation is enabled, we will get rviz data from this topic,
    // otherwise we will get out keyboard data. 
    // Then /vel is used for pushing data to the cartographer broadcaster node.
    cmdvelSub = n->subscribe("/cmd_vel",1,cmdvelDataCallback);
    pub_speed = n->advertise<nav_msgs::Odometry>("/vel",1);
    if(!w.EnableService(n, "keyboard")){
        ros::Subscriber keyboardSub;
        keyboardSub = n->subscribe(std::string(ROBOT_NAME)+std::string("keyboard/key"),1,keyboardDataCallback);
        while (keyboardSub.getNumPublishers() == 0) {}
        while (ros::ok()) {   
            ros::spinOnce();
            updateSpeed();
            if (w.ChecktimeStep())break;    
            ros::spinOnce();
        } 
    }
    w.Quit(n);
    return 0;
}