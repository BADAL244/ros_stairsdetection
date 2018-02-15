#include <vector>

#include "transform_helper.hpp"
#include "step.hpp"
#include "print_helpers.hpp"

void TransformHelper::getAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Step &step) {
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	pcl::PointXYZ min, max;
  feature_extractor.getAABB(min, max);

  feature_extractor.getOBB(step.min_point_OBB, step.max_point_OBB, step.position_OBB, step.rotational_matrix_OBB);

  //Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  //Eigen::Quaternionf quat (rotational_matrix_OBB);
  //viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

	// transform PCL points to ROS points
	geometry_msgs::Point min_r, max_r;
	transformPCLPointToROSPoint(min, min_r);
	transformPCLPointToROSPoint(max, max_r);

	step.setMinMax(min_r, max_r);
}

bool TransformHelper::transform(geometry_msgs::Point &point, std::string &target_frame, std::string &source_frame,
		float factor) {

	// Make sure that the transform buffer is not NULL
	if (m_tfBuffer == NULL) {
		ROS_ERROR("Transformlistener is NULL");
		return false;
	}

	geometry_msgs::TransformStamped ts;
	try {
		ts = m_tfBuffer->lookupTransform(target_frame.c_str(), source_frame.c_str(), ros::Time(0));
	} catch (tf2::TransformException &ex) {
		ROS_WARN("Failed to transform '%s' -> '%s'", target_frame.c_str(), source_frame.c_str());
		ROS_WARN("%s", ex.what());
		return false;
	}

	point.x = point.x + ts.transform.translation.x * factor;
	point.y = point.y + ts.transform.translation.y * factor;
	point.z = point.z + ts.transform.translation.z * factor;

	return true;
}

void TransformHelper::transformPCLPointToROSPoint(pcl::PointXYZ &input, geometry_msgs::Point &output) {
    //output.x = input.z;
    //output.y = input.x * (-1.f);
    //output.z = input.y * (-1.f);
    output.x = input.x;
    output.y = input.y;
    output.z = input.z;
}

/*
 * Get vertices of the rectangle
 *
 *  Max----------------P2
 *  |                   |
 *  |                   |
 *  P4----------------Min
 *
 */
void TransformHelper::buildStepFromAABB(Step &step, std::vector<geometry_msgs::Point> &points) {

  double avg_z = 0.5 * (step.getMin().z + step.getMax().z);

	// p2
  geometry_msgs::Point p;
  p.x = step.getMin().x;
  p.y = step.getMin().y;
  p.z = avg_z;
  points.push_back(p);
  p.x = step.getMax().x;
  points.push_back(p);
  p.y = step.getMax().y;
  points.push_back(p);
  p.x = step.getMin().x;
  points.push_back(p);
}
