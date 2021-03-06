#pragma once 

#include <iostream>

#include <functional>
#include <string> 
#include <vector>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>

#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"
#include <tf/transform_broadcaster.h> 


#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;
class PointCloudFusion
{
	

	private:
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_point_cloud;

		ros::Subscriber sub_point_cloud_;

		ros::Publisher pub_filtered_points_;
		
		Eigen::Matrix4f GlobalTransform;
		
		tf::StampedTransform transform;

		tf::TransformListener listener_;

		int counter;

		void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);

		void publish_pointcloud(const ros::Publisher &in_publihser, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr, const std_msgs::Header &in_header);
		
		void point_cloud_fusion(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_out, Eigen::Matrix4f &final_transform);

                tf::TransformBroadcaster br;
 	public:
		PointCloudFusion(ros::NodeHandle &nh);
		~PointCloudFusion();
		void Spin();
	
};
