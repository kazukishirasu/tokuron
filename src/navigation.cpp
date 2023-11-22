#include <ros/ros.h>
#include "ros/package.h"
#include "yaml-cpp/yaml.h"
#include <vector>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>


class Navigation{
    private:
        ros::NodeHandle nh;
        ros::Publisher goal_pub;
        ros::Subscriber pose_sub,
                        list_sub;
        struct Point {
            double x;
            double y;
            double z;
        };
        struct Spot {
            uint8_t num;
            std::string name;
            Point point;
        };
        std_msgs::UInt8MultiArray list_msg;
        std::vector<Spot> vec_spot;
        bool get_list = false;

    public:
        Navigation();
        void loop();
        void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void list_callback(const std_msgs::UInt8MultiArray& msg);
        void read_yaml();
        void send_goal(double, double, double);
};

Navigation::Navigation(){
    ROS_INFO("start navigation node");
    read_yaml();
    ROS_INFO("read yaml");
    list_sub = nh.subscribe("/list", 1, &Navigation::list_callback, this);
    pose_sub = nh.subscribe("/amcl_pose", 1, &Navigation::pose_callback, this);
}

void Navigation::loop(){
    if (get_list){
        double  x = vec_spot[list_msg.data[0]].point.x,
                y = vec_spot[list_msg.data[0]].point.y,
                z = vec_spot[list_msg.data[0]].point.z;
        send_goal(x, y, z);
        ROS_INFO("send goal");
    }else{
        ROS_INFO("waiting for message");
    }
}

void Navigation::pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg){
    double x, y, z;
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    z = msg.pose.pose.position.z;
    ROS_INFO("x = %f, y = %f, z = %f", x, y, z);
}

void Navigation::list_callback(const std_msgs::UInt8MultiArray& msg){
    int sum = msg.data.size();
    ROS_INFO("I subscribed [%i]", sum);
    for (int i = 0; i < sum; i++){
        ROS_INFO("[%i]:%d", i, msg.data[i]);
    }
    list_msg = msg;
    get_list = true;
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
            spot.num = spotNode["num"].as<uint8_t>();
            spot.name = spotNode["name"].as<std::string>();

            const YAML::Node& pointNode = spotNode["point"];
            spot.point.x = pointNode["x"].as<double>();
            spot.point.y = pointNode["y"].as<double>();
            spot.point.z = pointNode["z"].as<double>();

            vec_spot.push_back(spot);

            std::cout << "Num: " << spot.num << std::endl;
            std::cout << "Spot: " << spot.name << std::endl;
            std::cout << "  - Point: (" << spot.point.x << ", " << spot.point.y << ", " << spot.point.z << ")\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
    }
}

void Navigation::send_goal(double x, double y, double e){
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
 
    ros::Rate one_sec(1);
    one_sec.sleep();
     
    ros::Time time = ros::Time::now();
    geometry_msgs::PoseStamped goal_point;

    tf::Quaternion orientation=tf::createQuaternionFromRPY(0, 0, e);
 
    goal_point.pose.position.x = x;
    goal_point.pose.position.y = y;
    goal_point.pose.position.z =  0;
    goal_point.pose.orientation.x = orientation[0];
    goal_point.pose.orientation.y = orientation[1];
    goal_point.pose.orientation.z = orientation[2];
    goal_point.pose.orientation.w = orientation[3];
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