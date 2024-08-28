#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"
#include <cmath>
#include <typeinfo>

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

// Object
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path;
pathplanning::AstarConfig config;
pathplanning::Astar astar;
OccupancyGridParam OccGridParam;
Point startPoint, targetPoint;
Point start_pt, target_pt;

// Parameter
double InflateRadius;
bool map_flag;
bool compute_distanceMatrix;
bool startpoint_flag;
bool targetpoint_flag;
bool start_flag;
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
    // Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    // OccGridParam.Map2ImageTransform(src_point, startPoint);

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
    // Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    // OccGridParam.Map2ImageTransform(src_point, targetPoint);

    // Set flag
    targetpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

//    ROS_INFO("targetPoint: %f %f %d %d", msg.pose.position.x, msg.pose.position.y,
//             targetPoint.x, targetPoint.y);
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

    // Initial variables
    map_flag = false;
    startpoint_flag = false;
    targetpoint_flag = false;
    start_flag = false;

    //  Initial node
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    // ROS_INFO("Start astar node to compute distances between points!\n");
    
    // ROS_INFO("Total number of points = %d", total_points);
    
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

    float path_length = 0;
    int i = 0;
    int j = 0;
    compute_distanceMatrix = true;

    // Loop and wait for callback
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        if(map_flag == true && compute_distanceMatrix == true ){

            std::vector<std::vector<double>> distanceMatrix(total_points, std::vector<double>(total_points));
            

            for (i = 0; i < total_points; i++) {

                for (j = 0; j < total_points; j++) {

                    double start_time = ros::Time::now().toSec();

                    if(i!= j){

                        Point2d source_pt = Point2d(points[i].x, points[i].y);
                        OccGridParam.Map2ImageTransform(source_pt, start_pt);

                        Point2d goal_pt = Point2d(points[j].x, points[j].y);
                        OccGridParam.Map2ImageTransform(goal_pt, target_pt);

                        // Start planning path
                        vector<Point> PathList;
                        astar.PathPlanning(start_pt, target_pt, PathList);

                        if(!PathList.empty()){

                            path.header.stamp = ros::Time::now();
                            path.header.frame_id = "map";
                            path.poses.clear();
                            for(int i=0;i<PathList.size();i++){
                                
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
                            path_length = calculate_path_distance(path);
                            double end_time = ros::Time::now().toSec();

                            ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
                            ROS_INFO("Total path length %f ", path_length);
                            distanceMatrix[i][j] = path_length;
                        }
                        else{
                            ROS_ERROR("Can not find a valid path");
                        }

                    }

                    else{
                        distanceMatrix[i][j] = 0;
                        ROS_INFO("Same start and goal point!! Path length is 0");
                    }    
                }
            }

            mask_pub.publish(OccGridMask); 
            compute_distanceMatrix = false;  
            std::cout << "Distance Matrix:" << std::endl;
            for (int i = 0; i < total_points; ++i) {
                for (int j = 0; j < total_points; ++j) {
                    std::cout << distanceMatrix[i][j] << " ";
                }
                std::cout << std::endl;
            }

            // Write the distance matrix to a CSV file
            std::ofstream outputFile("/home/sutd/test_ws/src/Astar/scripts/distance_matrix.csv");
            if (!outputFile.is_open()) {
                std::cerr << "Error: Unable to open output file." << std::endl;
                return 1;
            }

            // for (int i = 0; i < total_points; ++i) {
            //     for (int j = 0; j < total_points; ++j) {
            //         outputFile << distanceMatrix[i][j];
            //         if (j < total_points - 1) outputFile << ",";
            //     }
            //     outputFile << std::endl;
            // }
            outputFile << "distance matrix,";
            for (int i = 0; i < total_points ; ++i) {
                outputFile << "Point " << i;
                if (i < total_points - 1) {
                    outputFile << ",";
                }
            }
            outputFile << std::endl;

            for (size_t i = 0; i < total_points; ++i) {
                outputFile << "Point " << i << ",";
                for (size_t j = 0; j < total_points; ++j) {
                    outputFile << distanceMatrix[i][j];
                    if (j < total_points - 1) {
                        outputFile << ",";
                    }
                }
                outputFile << std::endl;
            }

            outputFile.close();

            std::cout << "Distance matrix saved to distance_matrix.csv" << std::endl;
        
        }
        

        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}
