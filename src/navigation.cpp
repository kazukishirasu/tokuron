#include <ros/ros.h>
#include "ros/package.h"
#include "yaml-cpp/yaml.h"
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class Navigation{
    private:
        ros::NodeHandle nh;
        ros::Publisher goal_pub;
        ros::Subscriber pose_sub,
                        list_sub;
    public:
        Navigation();
        void loop();
        void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void list_callback(const std_msgs::UInt8MultiArray& msg);
        void read_yaml();
        void send_goal(double px, double py, double pz, double ow);
};

struct Point {
    double x;
    double y;
    double z;
};

struct Spot {
    std::string name;
    Point point;
};

Navigation::Navigation(){
    ROS_INFO("start navigation node");
    read_yaml();
    ROS_INFO("read yaml");
    list_sub = nh.subscribe("/list", 1, &Navigation::list_callback, this);
    pose_sub = nh.subscribe("/amcl_pose", 1, &Navigation::pose_callback, this);
}

void Navigation::loop(){
    send_goal(-0.6, 0.725, 0, 1);
    ROS_INFO("send goal");
}

void Navigation::pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg){
    double x, y, z;
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    z = msg.pose.pose.position.z;
    ROS_INFO("x = %f, y = %f, z = %f", x, y, z);
}

void Navigation::list_callback(const std_msgs::UInt8MultiArray& msg){
    int num = msg.data.size();
    ROS_INFO("I subscribed [%i]", num);
    for (int i = 0; i < num; i++){
        ROS_INFO("[%i]:%d", i, msg.data[i]);
    }
}

void Navigation::read_yaml(){
    std::string pkg_path = ros::package::getPath("tokuron");
    std::string yaml_path = "/spot/spot.yaml";
    pkg_path += yaml_path;
    YAML::Node config = YAML::LoadFile(pkg_path);
    ROS_INFO("%s", pkg_path.c_str());
    try {
        const YAML::Node& spots = config["spot"];
        for (const auto& spotNode : spots){
            Spot spot;
            spot.name = spotNode["name"].as<std::string>();

            const YAML::Node& pointNode = spotNode["point"];
            spot.point.x = pointNode["x"].as<double>();
            spot.point.y = pointNode["y"].as<double>();
            spot.point.z = pointNode["z"].as<double>();

            std::cout << "Spot: " << spot.name << std::endl;
            std::cout << "  - Point: (" << spot.point.x << ", " << spot.point.y << ", " << spot.point.z << ")\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
    }
}

void Navigation::send_goal(double px, double py, double pz, double ow){
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
 
    ros::Rate one_sec(1);
    one_sec.sleep();
     
    ros::Time time = ros::Time::now();
    geometry_msgs::PoseStamped goal_point;
 
    goal_point.pose.position.x = px;
    goal_point.pose.position.y = py;
    goal_point.pose.position.z =  pz;
    goal_point.pose.orientation.w = ow;
    goal_point.header.stamp = time;
    goal_point.header.frame_id = "map";
 
    goal_pub.publish(goal_point);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation");
    Navigation navigation;
    ros::Rate rate(10);
    while (ros::ok()){
        navigation.loop();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}