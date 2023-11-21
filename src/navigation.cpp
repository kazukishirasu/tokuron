#include <ros/ros.h>
#include "yaml-cpp/yaml.h"
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include "ros/package.h"

struct Point {
    double x;
    double y;
    double z;
};

struct Spot {
    std::string name;
    Point point;
};

class Navigation{
    private:
        ros::NodeHandle nh;
        ros::Publisher goal_pub;
        ros::Subscriber list_sub;
    public:
        Navigation();
        void list_callback(const std_msgs::UInt8MultiArray& msg);
        void read_yaml();
        void send_goal(double px, double py, double pz, double ow);
};

Navigation::Navigation(){
    list_sub = nh.subscribe("/list", 1, &Navigation::list_callback, this);
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
        const YAML::Node& spots = config["spots"];
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
    navigation.read_yaml();
    ros::spin();
    return 0;
}
