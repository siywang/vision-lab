
#ifndef FUTURE_POSE_ESTIMATOR_H
#define FUTURE_POSE_ESTIMATOR_H


#include <vector>
#include "apriltags2_ros/AprilTagDetectionArray.h"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

using namespace std;

#define PI 3.1415
#define LOOKAHEAD 2.0
#define FREQ 12
#define Wb 0.325


class Pose_Estimator {

private:
    // pure purist stuff
    float waypoint_x = 0;
    float waypoint_y = 0;
    vector<float> waypoint_data1;
    vector<float> waypoint_data2;
    vector<float> waypoint_data3;

    int last_index = -1;
    float rot_waypoint_x = 0;
    float rot_waypoint_y = 0;
    float angle_factor = 0.4;
    float steering_offset = -0.005;
    float steering_angle = 0;
    vector<vector<float>> waypoints;
    std::vector<std::vector<std::string> > dataList;
    vector<vector<float>> data_int;

    // weighted_model stuff
    float alpha = 0.7;
    float velocity = 0;

    float current_heading_angle = 0;
    float current_x = 0;
    float current_y = 0;
    float future_x = 0;
    float future_y = 0;

    geometry_msgs::Pose previous_pose;
    bool has_previous_pose = true;

    tf::TransformListener listener;

    apriltags2_ros::AprilTagDetectionArray old_data;
    geometry_msgs::PoseStamped before_transform;
    geometry_msgs::PoseStamped after_transform;

    void FuturePoseCallback(apriltags2_ros::AprilTagDetectionArray data);
    void PublishMarkers();

    visualization_msgs::Marker pose_marker;
    ros::Publisher pub_markers;
    ros::Subscriber sub;
    ros::Subscriber pf_sub;

public:
    Pose_Estimator(ros::NodeHandle &nh);

    static float Convert_to_Theta(geometry_msgs::Quaternion msg);

    void GetWaypoints();

    float PurePursuitAngle(float x, float y, float yaw);


};

#endif
