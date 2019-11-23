#include <iostream>
#include <unordered_map>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <math.h>
#include "ros/ros.h"
#include "vehicle_tracker.h"
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Pose.h>

// #include <experimental/filesystem>
// #include "apriltag_ros/AprilTagDetectionArray.h"

// #include <tf/transformations/euler_from_quaternion.h>


Pose_Estimator::Pose_Estimator(ros::NodeHandle &nh) {
        previous_pose.position.x = 0;
        previous_pose.position.y = 0;

        pose_marker.header.frame_id = "/map";
        pose_marker.ns = "/pose_predictions";
        pose_marker.type = visualization_msgs::Marker::SPHERE;
        pose_marker.action = visualization_msgs::Marker::ADD;
        pose_marker.scale.x = 0.2;
        pose_marker.scale.y = 0.2;
        pose_marker.scale.z = 0.1;
        pose_marker.color.a = 1.0;
        pose_marker.color.r = 1.0;
        pose_marker.color.g = 0.0;
        pose_marker.color.b = 0.0;
        pose_marker.id = 0;

        sub = nh.subscribe("/tag_detections", 10, &Pose_Estimator::FuturePoseCallback, this);
        pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/pose_predictions", 10);
}


void Pose_Estimator::GetWaypoints() {

        // File pointer
        std::ifstream myfile;
        myfile.open(
                "/home/siyan/SiyanWang_ws/src/f1_10/vehicle_tracker_prediction_skeleton/waypoints/levine-waypoints.csv");

        string line;
        string delimeter = ",";

        if (myfile.is_open()) {
                while (getline(myfile, line)) {
                        std::vector<std::string> vec;
                        boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
                        dataList.push_back(vec);
                }
                myfile.close();
        }

        unsigned long size = dataList.size();

        data_int = vector<vector<float > >(size, vector<float>(3, 0));

        for (unsigned int i = 0; i < size; i++) {
                for (unsigned int r = 0; r < 3; r++) {
                        // cout<<(stof(dataList[i][r]));
                        data_int[i][r] = stof(dataList[i][r]);
                }
        }

        waypoints = data_int;

}

float Pose_Estimator::PurePursuitAngle(float x, float y, float theta) {

        float csv_offset_x = 0;
        float csv_offset_y = 0;
        float csv_offset_theta = 0;

        //float waypoint_distance = sqrt( pow(x - waypoint_x, 2) + pow(y - waypoint_y, 2) );
        float distance_min = FLT_MAX;
        int ind_min = 0;

        for (int i = 0; i < waypoints.size(); i += 1) { //1
                auto distance = sqrt(
                        pow(x - (waypoints[i][0] - csv_offset_x), 2) + pow(y - (waypoints[i][1] - csv_offset_y), 2));
                rot_waypoint_x = (waypoints[i][0] - x) * cos(-theta) - (waypoints[i][1] - y) * sin(theta);
                rot_waypoint_y = (waypoints[i][0] - x) * sin(-theta) + (waypoints[i][1] - y) * cos(-theta);

                if (distance_min > distance && distance >= LOOKAHEAD &&
                    rot_waypoint_x > 0) { // LOOKAHEAD is defined in header file
                        distance_min = distance;
                        ind_min = i;
                }
        }

        last_index = ind_min;
        waypoint_x = waypoints[last_index][0];
        waypoint_y = waypoints[last_index][1];

        rot_waypoint_x = (waypoint_x - x) * cos(-theta) - (waypoint_y - y) * sin(-theta);
        rot_waypoint_y = (waypoint_x - x) * sin(-theta) + (waypoint_y - y) * cos(-theta);
        steering_angle = float(angle_factor * (2 * rot_waypoint_y) / (pow(rot_waypoint_x, 2) + pow(rot_waypoint_y, 2)));
        steering_angle += steering_offset;

        return steering_angle; //return steering angle to go to from pure pursuit
}


void Pose_Estimator::FuturePoseCallback(apriltags2_ros::AprilTagDetectionArray data) {
        //receiver information of poses from the april tag


        if (!data.detections.empty()) {
                //std::cout << "Receiving data" << std::endl;
                old_data = data;
        } else {
                //std::cout << ".................No Tag Detected....Use Previous Info" << std::endl;
                data = old_data;
        }

    before_transform.header.frame_id = "/other_base_link";
    //before_transform.header.frame_id = "/map";
    before_transform.pose = data.detections[0].pose.pose.pose;

    after_transform.header.frame_id = "/map";

    try {
        listener.waitForTransform("/map", "/other_base_link",
                                  ros::Time::now(), ros::Duration(3.0));
        listener.transformPose("/map", before_transform, after_transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    current_x = after_transform.pose.position.x;
    current_y = after_transform.pose.position.y;
    current_heading_angle = Convert_to_Theta(after_transform.pose.orientation); // heading with respect to map

    PublishMarkers();

}

float Pose_Estimator::Convert_to_Theta(const geometry_msgs::Quaternion msg) {

        tf::Quaternion quat; // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
        tf::quaternionMsgToTF(msg, quat);

        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // the tf::Quaternion has a method to acess roll pitch and yaw

        return (float) yaw; //theta is just the yaw angle
}

void Pose_Estimator::PublishMarkers() {
        visualization_msgs::MarkerArray viz_msg;

        // perform calculations and predictions by the linear and pure pursuit model. put them in a visualization message and then
//    if (has_previous_pose) {
//
//        double dist_x = current_x - previous_pose.position.x;
//        double dist_y = current_y - previous_pose.position.y;
//        velocity = sqrt(pow(dist_x, 2) + pow(dist_y, 2)) / (1.0/10.0); // velocity = distance * (frequency)
//
//        previous_pose.position.x = current_x;
//        previous_pose.position.y = current_y;
//
//        has_previous_pose = true;
//    } else {
//        ROS_INFO_STREAM("No Previous Pose");
//        has_previous_pose = false;
//    }

        //TODO: Keep velocity fixed ?
        float velocity = 2.0; // Using fixed velocity gives better visualization

        for (int i = 0; i <= 10; i++) {

                float pure_purist_steering_angle = PurePursuitAngle(current_x, current_y, current_heading_angle);
                float linear_heading_angle = current_heading_angle; // linear heading

                float angular_velocity = velocity / Wb * tan(pure_purist_steering_angle); // Wb --> Wheel Base
                // the above  equation is cited from https://slideplayer.com/slide/5362969/  (page 15)

                float angular_displacement = 0.1 * angular_velocity; // 0.1 is time-step
                float pure_purist_heading = current_heading_angle + angular_displacement;

                auto predicted_weighted_heading =
                        (pow(alpha, i) * linear_heading_angle + (1 - pow(alpha, i)) * pure_purist_heading);

                future_x = (float) (current_x + velocity * 0.1 * cos(predicted_weighted_heading)); // 0.1 is time-step
                future_y = (float) (current_y + velocity * 0.1 * sin(predicted_weighted_heading));

                pose_marker.pose.position.x = future_x;
                pose_marker.pose.position.y = future_y;
                //velocity = sqrt(pow(future_x - current_x, 2) + pow(future_y-current_y, 2)) / (1.0/10.0);

                current_x = future_x;
                current_y = future_y;
                current_heading_angle = predicted_weighted_heading;

                viz_msg.markers.push_back(pose_marker);
        }


        for (int i = 0; i < viz_msg.markers.size(); i++) {
                viz_msg.markers[i].id = i;
        }

        pub_markers.publish(viz_msg);
        viz_msg.markers.clear();

}


int main(int argc, char *argv[]) {


        ros::init(argc, argv, "planner_node");

        ros::NodeHandle nh;

        Pose_Estimator est_obj(nh);


        est_obj.GetWaypoints();

        ros::Rate loop_rate(10);


        while (ros::ok()) {

                ros::spinOnce();
                // est_obj.PublishMarkers();
                loop_rate.sleep();
        }


}
