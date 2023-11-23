#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <unistd.h>

class Navigation{
    private:
        ros::NodeHandle nh;
        ros::Publisher  goal_pub,
                        vel_pub;
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
        std::vector<Spot> vec_spot;
        std::vector<int> vec_array_msg;
        double px, py, pz;
        bool get_msg = false,
             reach_goal = false;

    public:
        Navigation();
        void loop();
        void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void list_callback(const std_msgs::UInt8MultiArray& msg);
        void read_yaml();
        void send_goal(double, double, double);
        void check_distance(double, double, double, double);
        void stop_vel();
};

Navigation::Navigation(){
    ROS_INFO("start navigation node");
    read_yaml();
    list_sub = nh.subscribe("/list", 1, &Navigation::list_callback, this);
    pose_sub = nh.subscribe("/mcl_pose", 1, &Navigation::pose_callback, this);
}

void Navigation::pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg){
    px = msg.pose.pose.position.x;
    py = msg.pose.pose.position.y;
    pz = msg.pose.pose.position.z;
    // ROS_INFO("x = %f, y = %f, z = %f", px, py, pz);
}

void Navigation::list_callback(const std_msgs::UInt8MultiArray& msg){
    vec_array_msg.clear();
    int sum = msg.data.size();
    ROS_INFO("I subscribed [%i]", sum);
    for (int i = 0; i < sum; i++){
        vec_array_msg.push_back(msg.data[i]);
        ROS_INFO("[%i]:%d", i, msg.data[i]);
    }
    get_msg = true;
}

void Navigation::loop(){
    static int spot_num = 0;
    if (reach_goal){
        stop_vel();
        spot_num++;
    }else if (get_msg){
        double  gx = vec_spot[vec_array_msg[spot_num]].point.x,
                gy = vec_spot[vec_array_msg[spot_num]].point.y,
                gz = vec_spot[vec_array_msg[spot_num]].point.z,
                d;
        if (spot_num > vec_array_msg.size()){
        }else if (spot_num == vec_array_msg.size()){
            send_goal(0.0, -0.7, -1.57);
            check_distance(0.0, -0.7, px, py);
        }else{
            send_goal(gx, gy, gz);
            check_distance(gx, gy, px, py);
        }
    }else{
        ROS_INFO("waiting for message");
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
        ROS_INFO("read yaml");
    } catch (const std::exception& e) {
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
    }
}

void Navigation::send_goal(double x, double y, double e){
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
 
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
    ROS_INFO("send goal");
}

void Navigation::check_distance(double gx, double gy, double px, double py){
    double distance;
    distance = sqrt(std::pow(px-gx, 2) + std::pow(py-gy, 2));
    ROS_INFO("distance : %f", distance);
    if (distance < 0.05){
        reach_goal = true;
    }
}

void Navigation::stop_vel(){
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.angular.z = 0;
    vel_pub.publish(vel);
    ROS_INFO("reached to goal");
    reach_goal = false;
    get_msg = true;
    sleep(2);
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