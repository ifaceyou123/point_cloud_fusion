#include "pcl_test_core.h"

PclTestCore::PclTestCore(ros::NodeHandle &nh){
	sub_point_cloud_ = nh.subscribe("/iris_1/camera/depth/points", 10, &PclTestCore::point_cb, this);
	
	pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);

	ros::spin();
}

PclTestCore::~PclTestCore(){}

void PclTestCore::Spin(){

}

void PclTestCore::publish_pointcloud(const ros::Publisher &in_publisher, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr, const std_msgs::Header &in_header)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	cloud_msg.header = in_header;
	in_publisher.publish(cloud_msg);
}

void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

	pcl::VoxelGrid<pcl::PointXYZRGB> vg;

	vg.setInputCloud(current_pc_ptr);
	vg.setLeafSize(0.02f, 0.02f, 0.02f);
	vg.filter(*filtered_pc_ptr);

	publish_pointcloud(pub_filtered_points_, filtered_pc_ptr, in_cloud_ptr->header);

}


