#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"
#include <cmath>

using namespace cv;
using namespace std;

struct coordinate {
    double x;
    double y;
};


//-------------------------------- Global variables ---------------------------------//
// Subscriber
ros::Subscriber map_sub;
ros::Subscriber startPoint_sub;
ros::Subscriber targetPoint_sub;
// Publisher
ros::Publisher mask_pub;
ros::Publisher path_pub;
ros::Publisher combined_path_pub;
ros::Publisher marker_pub;
ros::Publisher text_marker_array_pub;

// Object
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path, combined_path;
visualization_msgs::Marker marker, text_marker;
// Marker array to store text markers
visualization_msgs::MarkerArray text_marker_array;
geometry_msgs::Point temp_pt;
pathplanning::AstarConfig config;
pathplanning::Astar astar;
OccupancyGridParam OccGridParam;
Point startPoint, targetPoint;
Point start_pt, target_pt;

// Parameter
double InflateRadius;
bool map_flag;
bool startpoint_flag;
bool targetpoint_flag;
bool start_flag;
bool compute_TotalPath;
int rate;

//-------------------------------- Callback function ---------------------------------//
void MapCallback(const nav_msgs::OccupancyGrid& msg)
{
    // Get parameter
    OccGridParam.GetOccupancyGridParam(msg);

    // Get map
    int height = OccGridParam.height;
    int width = OccGridParam.width;
    int OccProb;
    Mat Map(height, width, CV_8UC1);
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = msg.data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
            // The origin of the OccGrid is on the bottom left corner of the map
            Map.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }

    // Initial Astar
    Mat Mask;
    config.InflateRadius = round(InflateRadius / OccGridParam.resolution);
    astar.InitAstar(Map, Mask, config);

    // Publish Mask
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "map";
    OccGridMask.info = msg.info;
    OccGridMask.data.clear();
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = Mask.at<uchar>(height-i-1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }

    // Set flag
    map_flag = true;
    startpoint_flag = false;
    targetpoint_flag = false;
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, startPoint);

    // Set flag
    startpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

//    ROS_INFO("startPoint: %f %f %d %d", msg.pose.pose.position.x, msg.pose.pose.position.y,
//             startPoint.x, startPoint.y);
}

void TargetPointtCallback(const geometry_msgs::PoseStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint);

    // Set flag
    targetpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

//    ROS_INFO("targetPoint: %f %f %d %d", msg.pose.position.x, msg.pose.position.y,
//             targetPoint.x, targetPoint.y);
}

void combinedPathCallback(const nav_msgs::Path path_msg) {
        // Add received path to combined path

        combined_path.header.frame_id = "map";

        for (const auto& pose : path_msg.poses) {
            combined_path.poses.push_back(pose);
        }

        // Publish the combined path
        combined_path_pub.publish(combined_path);
}

float calculate_distance(const geometry_msgs::PoseStamped point1, const geometry_msgs::PoseStamped point2){
    
    // Euclidean distance between two points in 2D space
    return sqrt(pow((point2.pose.position.x - point1.pose.position.x), 2) + pow((point2.pose.position.y - point1.pose.position.y), 2)); 
}

float calculate_path_distance(const nav_msgs::Path path_msg){

    float total_distance = 0.0;
    size_t length = path_msg.poses.size();

    for (int i = 0; i < length -1 ; i++){
        total_distance += calculate_distance(path_msg.poses[i], path_msg.poses[i+1]);
    }

    return total_distance;
}

//-------------------------------- Main function ---------------------------------//
int main(int argc, char * argv[])
{      
    std::vector<coordinate> points;
    // Open the CSV file
    std::ifstream file("/home/sutd/test_ws/src/Astar/scripts/point_profile.csv");
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file." << std::endl;
        return 1;
    }

    // Read each line and extract x, y coordinates
    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {

        std::stringstream ss(line);

        std::string cell;
        coordinate point;

        if (std::getline(ss, cell, ',') && std::istringstream(cell) >> point.x) {
            // Extract the y coordinate from the second column
            if (std::getline(ss, cell, ',') && std::istringstream(cell) >> point.y) {
                // Successfully extracted both coordinates, add the point to the vector
                points.push_back(point);
            } 
            
            else {
                std::cerr << "Error: Invalid data format in Y column." << std::endl;
            }
        } 
        else {
            std::cerr << "Error: Invalid data format in X column." << std::endl;
        }
    }
    file.close();
    //calculate the number of total points
    int total_points = points.size();

    std::string filename = "/home/sutd/test_ws/src/Astar/scripts/optimed_path.csv";

    // Vector to store the optimized sequence
    std::vector<int> optimized_sequence;

    // Open the CSV file
    std::ifstream file_opt(filename);

    // Check if the file is opened successfully
    if (!file_opt.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return 1; // Return error code
    }

    std::getline(file_opt, line);

    // Read each line of the CSV file
    while (std::getline(file_opt, line)) {
        // Create a string stream from the line
        std::istringstream iss(line);
        std::string token;
        float h;
        // Read each token separated by comma
        if (std::getline(iss, token, ',') && std::istringstream(token) >> h) {
            // Convert token to integer and store in vector
            optimized_sequence.push_back(h);
        }
    }

    std::cout<< "optimized_sequence len = "<< optimized_sequence.size() << std::endl; 
    
    // Close the file
    file_opt.close();

    //  Initial node
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start astar node!\n");

    // Initial variables
    map_flag = false;
    startpoint_flag = false;
    targetpoint_flag = false;
    start_flag = false;

    // Parameter
    nh_priv.param<bool>("Euclidean", config.Euclidean, true);
    nh_priv.param<int>("OccupyThresh", config.OccupyThresh, -1);
    nh_priv.param<double>("InflateRadius", InflateRadius, -1);
    nh_priv.param<int>("rate", rate, 10);

    // Subscribe topics
    map_sub = nh.subscribe("map", 10, MapCallback);
    startPoint_sub = nh.subscribe("initialpose", 10, StartPointCallback);
    targetPoint_sub = nh.subscribe("move_base_simple/goal", 10, TargetPointtCallback);
    
    // Advertise topics
    mask_pub = nh.advertise<nav_msgs::OccupancyGrid>("mask", 1);
    path_pub = nh.advertise<nav_msgs::Path>("nav_path", 10);
    combined_path_pub = nh.advertise<nav_msgs::Path>("combined_path", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("waypoint_marker", 10);
    text_marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_id_marker", 10);

    float path_length = 0;
    compute_TotalPath = true;

    marker.header.frame_id = "map"; // Assuming coordinates are in the map frame
    marker.header.stamp = ros::Time::now();
    marker.ns = "waypoints";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.7; // Point size
    marker.scale.y = 0.7; // Point size
    marker.color.a = 1.0; // Alpha (transparency)
    marker.color.r = 0.0; // Red color
    marker.color.g = 0.0; // Green color
    marker.color.b = 1.0; // Blue color

    // Loop and wait for callback
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        if(map_flag == true && compute_TotalPath == true ){

            for (size_t i = 0; i < points.size(); ++i) {
                temp_pt.x = points[i].x;
                temp_pt.y = points[i].y;
                temp_pt.z = 0.0;
                marker.points.push_back(temp_pt);
        
                // Create text label for the waypoint
                visualization_msgs::Marker text_marker;
                text_marker.header.frame_id = "map"; // Assuming points are in the map frame
                text_marker.header.stamp = marker.header.stamp;
                text_marker.ns = "waypoint_labels";
                text_marker.id = i;
                text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text_marker.action = visualization_msgs::Marker::ADD;
                text_marker.scale.z = 1.5; // Text height
                text_marker.color.a = 1.0;
                text_marker.color.r = 1.0;
                text_marker.color.g = 1.0;
                text_marker.color.b = 0.0;
                text_marker.pose.position.x = temp_pt.x - 1.5;
                text_marker.pose.position.y = temp_pt.y -1.5;
                text_marker.pose.position.z = 1.8; // Offset for text above the point
                text_marker.pose.orientation.w = 1.0;
                text_marker.text = "waypoint_" + std::to_string(i);

                // Add text marker to marker array
                text_marker_array.markers.push_back(text_marker);
            }

            for(int i = 0; i < optimized_sequence.size() -1 ; i++){

                double start_time = ros::Time::now().toSec();
                // Start planning path
                Point2d source_pt = Point2d(points[optimized_sequence[i]].x, points[optimized_sequence[i]].y);
                OccGridParam.Map2ImageTransform(source_pt, start_pt);
                Point2d goal_pt = Point2d(points[optimized_sequence[i+1]].x, points[optimized_sequence[i+1]].y);
                OccGridParam.Map2ImageTransform(goal_pt, target_pt);
                // Start planning path
                vector<Point> PathList;
                astar.PathPlanning(start_pt, target_pt, PathList);


                if(!PathList.empty())
                {
                    path.header.stamp = ros::Time::now();
                    path.header.frame_id = "map";
                    path.poses.clear();
                    for(int i=0;i<PathList.size();i++)
                    {
                        Point2d dst_point;
                        OccGridParam.Image2MapTransform(PathList[i], dst_point);

                        geometry_msgs::PoseStamped pose_stamped;
                        pose_stamped.header.stamp = ros::Time::now();
                        pose_stamped.header.frame_id = "map";
                        pose_stamped.pose.position.x = dst_point.x;
                        pose_stamped.pose.position.y = dst_point.y;
                        pose_stamped.pose.position.z = 0;
                        path.poses.push_back(pose_stamped);
                    }
                    path_pub.publish(path);
                    combinedPathCallback(path);

                    path_length = calculate_path_distance(path);
                    double end_time = ros::Time::now().toSec();

                    ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
                    ROS_INFO("Total path length %f ", path_length);
                }
                else
                {
                    ROS_ERROR("Can not find a valid path");
                }
            }
      
        // mask_pub.publish(OccGridMask);
        compute_TotalPath = false;
        }

        mask_pub.publish(OccGridMask);
        combined_path_pub.publish(combined_path);
        // Publish the marker and text marker array
        marker_pub.publish(marker);
        text_marker_array_pub.publish(text_marker_array);

        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}
