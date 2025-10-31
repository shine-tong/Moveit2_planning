#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>
// #include <moveit_msgs/msg/attached_collision_object.hpp>
// #include <moveit_msgs/msg/collision_object.hpp>

using namespace std;

#define pi 3.1415926

void euler2quat(float phi, float theta, float psi, float quat[4]) {
    float c1 = cos(phi / 2);
    float c2 = cos(theta / 2);
    float c3 = cos(psi / 2);
    float s1 = sin(phi / 2);
    float s2 = sin(theta / 2);
    float s3 = sin(psi / 2);
    quat[0] = c1*c2*c3 + s1*s2*s3;
    quat[1] = s1*c2*c3 - c1*s2*s3;
    quat[2] = c1*s2*c3 + s1*c2*s3;
    quat[3] = c1*c2*s3 - s1*s2*c3;
}

vector<string> splitStr(const string& src, const string& delimiter) {
    vector<string> vtStr;
    if (src == "") return vtStr;
    if (delimiter == "") { vtStr.push_back(src); return vtStr; }

    size_t startPos = 0;
    auto index = src.find_first_of(delimiter);
    while (index != string::npos) {
        auto str = src.substr(startPos, index - startPos);
        if (str != "") vtStr.push_back(str);
        startPos = index + 1;
        index = src.find_first_of(delimiter, startPos);
    }
    auto str = src.substr(startPos);
    if (str != "") vtStr.push_back(str);
    return vtStr;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("visual_node");

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;

    geometry_msgs::msg::Pose target_pose1;
    geometry_msgs::msg::Pose target_pose2;
    vector<geometry_msgs::msg::Pose> waypoints[20];

    string f_path;
    node->declare_parameter("folder_path", "");
    node->get_parameter("folder_path", f_path);
    f_path = f_path + "/result.txt";

    ifstream file(f_path);
    int num = 0;
    string line;
    vector<string> l;

    if (file.is_open()) {
        while (getline(file, line)) {
            l = splitStr(line, "sp,/");
            if(l.size() < 14) continue;

            target_pose1.position.x = stod(l[0]) / 1000.0;
            target_pose1.position.y = stod(l[1]) / 1000.0;
            target_pose1.position.z = stod(l[2]) / 1000.0;

            target_pose2.position.x = stod(l[3]) / 1000.0;
            target_pose2.position.y = stod(l[4]) / 1000.0;
            target_pose2.position.z = stod(l[5]) / 1000.0;

            target_pose1.orientation.x = stod(l[6]);
            target_pose1.orientation.y = stod(l[7]);
            target_pose1.orientation.z = stod(l[8]);
            target_pose1.orientation.w = stod(l[9]);

            target_pose2.orientation.x = stod(l[10]);
            target_pose2.orientation.y = stod(l[11]);
            target_pose2.orientation.z = stod(l[12]);
            target_pose2.orientation.w = stod(l[13]);

            waypoints[num].push_back(target_pose1);
            waypoints[num].push_back(target_pose2);
            num++;
        }
        file.close();
    }

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "robot", rvt::WHITE, rvt::XLARGE);
    for(int k = 0; k < num; k++) {
        visual_tools.publishPath(waypoints[k], rvt::PINK , rvt::XXXSMALL);
        for (size_t i = 0; i < waypoints[k].size(); ++i)
            visual_tools.publishAxisLabeled(waypoints[k][i], 
                "point" + to_string(k) + "  "+ to_string(waypoints[k][i].position.x)
                + "  "+ to_string(waypoints[k][i].position.y) 
                + "  "+ to_string(waypoints[k][i].position.z),
                rvt::XXXSMALL);
        visual_tools.trigger();
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}