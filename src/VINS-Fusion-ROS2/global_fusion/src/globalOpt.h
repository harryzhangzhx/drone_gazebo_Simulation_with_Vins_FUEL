/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "LocalCartesian.hpp"
#include "tic_toc.h"
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <queue>

using namespace std;

class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
	nav_msgs::msg::Path global_path;

private:
	void GPS2XYZ(double latitude, double longitude, double altitude, double *xyz);
	void optimize();
	void updateGlobalPath();

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;
	map<double, vector<double>> globalPoseMap;
	map<double, vector<double>> GPSPositionMap;
	bool initGPS;
	bool newGPS;
	GeographicLib::LocalCartesian geoConverter;
	std::mutex mPoseMap;
	Eigen::Matrix4d WGPS_T_WVIO;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	std::thread threadOpt;
};
