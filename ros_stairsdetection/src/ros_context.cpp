#include "ros_context.hpp"

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>

void ROSContext::init(int argc, char **argv, void (*callback)(const sensor_msgs::PointCloud2ConstPtr&),
		bool (*exportStairs)(ros_stairsdetection::ExportStairs::Request&,
			ros_stairsdetection::ExportStairs::Response&),
		bool (*importStairs)(ros_stairsdetection::ImportStairs::Request&,
			ros_stairsdetection::ImportStairs::Response&),
		bool (*clearStairs)(ros_stairsdetection::ClearStairs::Request&,
			ros_stairsdetection::ClearStairs::Response&)) {

	/*
	 * load parameters from launch file
	 */
	std::string inputSetting;
	std::string stepsSetting;
	std::string stairwaysSetting;
	bool useSampleDataSetting;

	ros::init(argc, argv, "stairsdetection");
	ros::NodeHandle nh;

	ros::param::get("~input",  inputSetting);
	ros::param::get("~steps", stepsSetting);
	ros::param::get("~stairways", stairwaysSetting);

	ros::param::get("~publish_steps", m_publishStepsSetting);
	ros::param::get("~publish_stairways", m_publishStairwaysSetting);

	ros::param::get("~segmentation_iterations", m_segmentationIterationSetting);
	ros::param::get("~segmentation_threshold", m_segmentationThresholdSetting);

	ros::param::get("~min_step_width", m_minStepWidthSetting);
  ros::param::get("~max_step_width", m_maxStepWidthSetting);
	ros::param::get("~min_step_height", m_minStepHeightSetting);
	ros::param::get("~max_step_height", m_maxStepHeightSetting);

	ros::param::get("~camera_frame", m_cameraFrameSetting);
	ros::param::get("~robot_frame", m_robotFrameSetting);
	ros::param::get("~world_frame", m_worldFrameSetting);
	ros::param::get("~namespace", m_namespaceSetting);

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	m_th = TransformHelper(m_cameraFrameSetting, m_robotFrameSetting, m_worldFrameSetting, &tfBuffer);

	/*
	 * Init subscriber and listener
	 */
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(inputSetting.c_str(), 1, callback);
	m_pubSteps = nh.advertise<visualization_msgs::MarkerArray>(stepsSetting.c_str(), 0);
	m_pubStairways = nh.advertise<visualization_msgs::MarkerArray>(stairwaysSetting.c_str(), 0);

	/*
	 * Init services to import/export stairways
	 */
	m_exportService = nh.advertiseService("export_stairs", exportStairs);
	m_importService = nh.advertiseService("import_stairs", importStairs);
	m_clearService  = nh.advertiseService("clear_stairs", clearStairs);

	ros::spin();
}

void ROSContext::buildRosMarkerSteps(visualization_msgs::MarkerArray &marker_array, std::vector<Step> &steps,
	double (&color)[3]) {

  visualization_msgs::Marker marker;
	marker.header.frame_id = m_cameraFrameSetting.c_str();
	marker.header.stamp = ros::Time::now();
	marker.ns = m_namespaceSetting.c_str();
	marker.id = 0;
	marker.lifetime = ros::Duration();

  marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 1.0f;
  marker.scale.y = 1.0f;
  marker.scale.z = 1.0f;
	marker.color.r = color[0];
	marker.color.g = color[1];
	marker.color.b = color[2];
  marker.color.a = 0.3;

	for (std::vector<Step>::iterator it = steps.begin(); it != steps.end(); it++) {

    marker.id = marker_array.markers.size();
		std::vector<geometry_msgs::Point> points;
		m_th.buildStepFromAABB(*it, points);

		geometry_msgs::Point p1 = points.at(0);
		geometry_msgs::Point p2 = points.at(1);
		geometry_msgs::Point p3 = points.at(2);
		geometry_msgs::Point p4 = points.at(3);
/*
		marker.points.push_back(p1);
		marker.points.push_back(p2);
		marker.points.push_back(p2);
		marker.points.push_back(p3);
		marker.points.push_back(p3);
		marker.points.push_back(p4);
		marker.points.push_back(p4);
    marker.points.push_back(p1);*/

    marker.pose.position.x = 0.25*(p1.x + p2.x + p3.x + p4.x);
    marker.pose.position.y = 0.25*(p1.y + p2.y + p3.y + p4.y);
    marker.pose.position.z = 0.25*(p1.z + p2.z + p3.z + p4.z);
    marker.scale.x = std::abs(p1.x - marker.pose.position.x);
    marker.scale.y = std::abs(p1.y - marker.pose.position.y);
    marker.scale.z = std::abs(p1.z - marker.pose.position.z);
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker_array.markers.push_back(marker);
	}
}

/**
 * Publish stairways
 */
void ROSContext::publishStairways(std::vector<Stairway> &stairway) {
	
	// Contains 
	visualization_msgs::MarkerArray markerArray;

	//
	for (std::vector<Stairway>::iterator it = stairway.begin(); it != stairway.end(); it++) {
		visualization_msgs::Marker marker;
		double color[3];
		color[0] = color[2] = 0.f;
		color[1] = 1.f;

    buildRosMarkerSteps(markerArray, it->getSteps(), color);
    //markerArray.markers.push_back(marker);
	}

	m_pubStairways.publish(markerArray);
}

void ROSContext::publishSteps(std::vector<Step> &steps) {
	visualization_msgs::MarkerArray markerArray;
	visualization_msgs::Marker marker;
	double color[3];
	color[0] = color[1] = 0.f;
	color[2] = 1.f;

  buildRosMarkerSteps(markerArray, steps, color);
  //markerArray.markers.push_back(marker);
	m_pubSteps.publish(markerArray);
}
