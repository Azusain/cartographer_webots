#include <signal.h>
#include <geometry_msgs/Twist.h> 
#include <webots_ros.h> 

ros::NodeHandle *n;

// global settings for the motors in webots.
const int TIME_STEP = 32;                   
const int NMOTORS = 2;                    
const float MAX_SPEED = 2.0;                
const std::string ROBOT_NAME = "robot/";    
double speeds[NMOTORS]={0.0,0.0};           

static const char *motorNames[NMOTORS] ={"left_motor", "right_motor"};// 控制位置电机名称

static int controllerCount;
static std::vector<std::string> controllerList; 

Webots w = Webots(TIME_STEP,ROBOT_NAME);

// update the speed of the motors in webots.
void updateSpeed() {   
    for (int i = 0; i < NMOTORS; ++i) {
        w.SetMotorsVelocity(n, motorNames[i], -speeds[i]);
    }
}

// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    controllerList.push_back(name->data);//将控制器名加入到列表中
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

void quit(int sig) {
    w.Quit(n);
}

// calculate the velocity for controling the robots.
void cmdvelDataCallback(const geometry_msgs::Twist::ConstPtr &value){
    float linear_temp=0, angular_temp=0;
    float L = 0.6;
    angular_temp = value->angular.z ;
    linear_temp = value->linear.x ;

    speeds[0]  = 10.0*(2.0*linear_temp - L*angular_temp)/2.0;
    speeds[1]  = 10.0*(2.0*linear_temp + L*angular_temp)/2.0;
    updateSpeed();
    ROS_INFO("left_vel:%lf,  right_vel:%lf", speeds[0], speeds[1]);
}

int main(int argc, char **argv) {
    std::string controllerName;
    ros::init(argc, argv, "robot_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;

    signal(SIGINT, quit);
    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    w.Init(n, nameSub, controllerCount, controllerList);
    w.InitMotors(n, motorNames, NMOTORS);

    ros::Subscriber cmdvelSub;
    // subscribe to the message from the navigation node.
    cmdvelSub = n->subscribe("/cmd_vel",1,cmdvelDataCallback);
    while (cmdvelSub.getNumPublishers() == 0) {}
    while (ros::ok()) {   
        if (w.ChecktimeStep())break;
        ros::spinOnce();
    } 
    w.Quit(n);
    return 0;
}
