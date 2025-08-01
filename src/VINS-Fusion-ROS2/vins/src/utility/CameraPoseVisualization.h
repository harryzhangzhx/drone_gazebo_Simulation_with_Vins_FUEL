/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class CameraPoseVisualization
{
public:
	std::string m_marker_ns;

	CameraPoseVisualization(float r, float g, float b, float a);

	void setImageBoundaryColor(float r, float g, float b, float a = 1.0);
	void setOpticalCenterConnectorColor(float r, float g, float b, float a = 1.0);
	void setScale(double s);
	void setLineWidth(double width);

	void add_pose(const Eigen::Vector3d &p, const Eigen::Quaterniond &q);
	void reset();

	void publish_by(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub, const std_msgs::msg::Header &header);
	void add_edge(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1);
	void add_loopedge(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1);

private:
	std::vector<visualization_msgs::msg::Marker> m_markers;
	std_msgs::msg::ColorRGBA m_image_boundary_color;
	std_msgs::msg::ColorRGBA m_optical_center_connector_color;
	double m_scale;
	double m_line_width;

	static const Eigen::Vector3d imlt;
	static const Eigen::Vector3d imlb;
	static const Eigen::Vector3d imrt;
	static const Eigen::Vector3d imrb;
	static const Eigen::Vector3d oc;
	static const Eigen::Vector3d lt0;
	static const Eigen::Vector3d lt1;
	static const Eigen::Vector3d lt2;
};
