#include "icp_fusion_core.h"

PointCloudFusion::PointCloudFusion(ros::NodeHandle &nh):icp_result(new pcl::PointCloud<pcl::PointXYZRGB>),counter(0){
	
//subscribe sensor_msgs::pointcloud2
	sub_point_cloud_ = nh.subscribe("/iris_1/camera/depth/points", 10, &PointCloudFusion::point_cb, this);

//publish the filtered point cloud to ros
	pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);

	ros::spin();
}

PointCloudFusion::~PointCloudFusion(){}

void PointCloudFusion::Spin(){

}

//publish different point cloud topic
void PointCloudFusion::publish_pointcloud(const ros::Publisher &in_publisher, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_to_publish, const std_msgs::Header &in_header)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*input_cloud_to_publish, cloud_msg);
	cloud_msg.header = in_header;
	in_publisher.publish(cloud_msg);
}

void PointCloudFusion::point_cloud_fusion(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr out)
{
//use ICP algorithm to fusion the point cloud 
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//Set the convergence judgment condition. The smaller the smaller the precision, the slower the convergence.
	icp.setTransformationEpsilon (1e-6);
//Set the maximum distance between two correspondences (src<->tgt) to 10cm
	icp.setMaxCorrespondenceDistance (0.1);
//Difference between two iterations before and after
	icp.setEuclideanFitnessEpsilon (0.5);
//set the maximum number of iterations
	icp.setMaximumIterations (10); 


	icp.setInputSource (in);   // set source point cloud
	icp.setInputTarget (out); //set target point cloud

	icp.align (*icp_result);
}

// call back the point cloud and filter it, then use ICP to fusion point cloud to generate map 
void PointCloudFusion::point_cb(const sensor_msgs::PointCloud2ConstPtr & input_cloud){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZRGB>);

// transform the pointcloud from ros type to pcl type
	pcl::fromROSMsg(*input_cloud, *current_point_cloud);


//use voxel to filter the pointcloud, which from the sensor_msgs::pointcloud2
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	vg.setInputCloud(current_point_cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f); 
	vg.filter(*filtered_point_cloud);
	
	counter = counter + 1 ;

	if(counter == 1)
	{
		cout<<"has been "<<endl;
		icp_result = filtered_point_cloud;
		cout<<"has been processed"<<endl;
	}
	else
	{
		point_cloud_fusion(filtered_point_cloud, icp_result);
	}
	if(counter % 10 == 0)
	{
		publish_pointcloud(pub_filtered_points_, icp_result, input_cloud->header);
	}
	
}










































