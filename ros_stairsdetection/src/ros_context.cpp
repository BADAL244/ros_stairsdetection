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

  ros::param::get("~min_step_num_points", m_minNumPointsSetting);
  ros::param::get("~max_step_num_points", m_maxNumPointsSetting);
  ros::param::get("~min_step_depth", m_minStepDepthSetting);
  ros::param::get("~max_step_depth", m_maxStepDepthSetting);
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
    m_pubCloud = nh.advertise<sensor_msgs::PointCloud2>("steps_cloud", 0);

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

    //marker.header.frame_id = m_cameraFrameSetting.c_str();
    marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();
	marker.ns = m_namespaceSetting.c_str();
	marker.id = 0;
	marker.lifetime = ros::Duration();

    marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 0.05f;
	marker.color.r = color[0];
	marker.color.g = color[1];
	marker.color.b = color[2];
    marker.color.a = 0.2;

  for (std::vector<Step>::iterator it = steps.begin(); it != steps.end(); it++) {
    Eigen::Vector3f position (it->position_OBB.x, it->position_OBB.y, it->position_OBB.z);
    Eigen::Quaternionf quat (it->rotational_matrix_OBB);
    marker.id = marker_array.markers.size();
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = quat.w();
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.scale.x = it->max_point_OBB.x - it->min_point_OBB.x;
    marker.scale.y = it->max_point_OBB.y - it->min_point_OBB.y;
    marker.scale.z = it->max_point_OBB.z - it->min_point_OBB.z;
    marker_array.markers.push_back(marker);

    marker.ns = "lines";
    marker.id = marker_array.markers.size();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.01;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.8;

    geometry_msgs::Point p;
    p.x = it->min_point_OBB.x;
    p.y = it->min_point_OBB.y;
    p.z = 0.0;
    marker.points.push_back(p);

    p.x = it->max_point_OBB.x;
    marker.points.push_back(p);

    p.y = it->max_point_OBB.y;
    marker.points.push_back(p);

    p.x = it->min_point_OBB.x;
    marker.points.push_back(p);

    marker_array.markers.push_back(marker);

    p.y = it->min_point_OBB.y;
    marker.points.push_back(p);

    marker_array.markers.push_back(marker);

    marker.points.clear();


	}
}

/**
 * Publish stairways
 */
void ROSContext::publishStairways(std::vector<Stairway> &stairway) {
	
	// Contains 
  visualization_msgs::MarkerArray markerArray;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = m_namespaceSetting.c_str();
  marker.id = 0;
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::DELETEALL;

  for (std::vector<Stairway>::iterator it = stairway.begin(); it != stairway.end(); it++) {
    visualization_msgs::Marker marker;
    double color[3];
    color[0] = color[2] = 0.f;
    color[1] = 1.f;

    buildRosMarkerSteps(markerArray, it->getSteps(), color);
	}

	m_pubStairways.publish(markerArray);
}

void ROSContext::publishSteps(std::vector<Step> &steps) {
  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = m_namespaceSetting.c_str();
  marker.id = 0;
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::DELETEALL;
  markerArray.markers.push_back(marker);
  double color[3];
  color[0] = color[1] = 0.f;
  color[2] = 1.f;

  buildRosMarkerSteps(markerArray, steps, color);
	m_pubSteps.publish(markerArray);
}

bool ROSContext::cloudRequested() const
{
  return (m_pubCloud.getNumSubscribers() > 0);
}

void ROSContext::publishCloud(const sensor_msgs::PointCloud2& cloud)
{
  m_pubCloud.publish(cloud);
}
