#pragma once 

#include <functional>
#include <string> 

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>


class PclTestCore
{
	

	private:
		ros::Subscriber sub_point_cloud_;

		ros::Publisher pub_filtered_points_;

		void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);

		void publish_pointcloud(const ros::Publisher &in_publihser, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr, const std_msgs::Header &in_header);
		
		void point_cloud_fusion(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

 	public:
		PclTestCore(ros::NodeHandle &nh);
		~PclTestCore();
		void Spin();
	
};
