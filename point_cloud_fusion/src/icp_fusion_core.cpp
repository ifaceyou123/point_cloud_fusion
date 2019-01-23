#include "icp_fusion_core.h"

PointCloudFusion::PointCloudFusion(ros::NodeHandle &nh):icp_result(new pcl::PointCloud<pcl::PointXYZRGB>),counter(0),GlobalTransform(Eigen::Matrix4f::Identity()){
	
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

void PointCloudFusion::point_cloud_fusion(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr out, pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_out, Eigen::Matrix4f &final_transform)
{
//use ICP algorithm to fusion the point cloud 
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//Set the convergence judgment condition. The smaller the smaller the precision, the slower the convergence.
	icp.setTransformationEpsilon (1e-6);
//Set the maximum distance between two correspondences (src<->tgt) to 10cm
	icp.setMaxCorrespondenceDistance (0.1);
//Difference between two iterations before and after
	//icp.setEuclideanFitnessEpsilon (0.5);
//set the maximum number of iterations
	icp.setMaximumIterations (50); 
// Run the same optimization in a loop 
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	icp.setInputSource (in);   // set source point cloud
	icp.setInputTarget (out); //set target point cloud

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
	icp.align (*result);

//accumulate transformation between each Iteration
	Ti = icp.getFinalTransformation () * Ti;

// Get the transformation from target to source
	targetToSource = Ti.inverse();

// Transform target back in source frame
	pcl::transformPointCloud (*out, *temp_out, targetToSource);

	*temp_out += *in;

	final_transform = targetToSource;

	//icp_result = result;
	std::cout <<  " score: " << icp.getFitnessScore() << std::endl;
}

// call back the point cloud and filter it, then use ICP to fusion point cloud to generate map 
void PointCloudFusion::point_cb(const sensor_msgs::PointCloud2ConstPtr & input_cloud){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_result(new pcl::PointCloud<pcl::PointXYZRGB>);

	Eigen::Matrix4f pairTransform;
// transform the pointcloud from ros type to pcl type
	pcl::fromROSMsg(*input_cloud, *current_point_cloud);


//use voxel to filter the pointcloud, which from the sensor_msgs::pointcloud2
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	vg.setInputCloud(current_point_cloud);
	vg.setLeafSize(0.07f, 0.07f, 0.07f); 
	vg.filter(*filtered_point_cloud);
	
	counter = counter + 1 ;

	if(counter == 1)
	{
		icp_result = filtered_point_cloud; //save the first frame as the source frame
		cout<<"has been processed"<<endl;
	}
	else
	{
		point_cloud_fusion(icp_result, filtered_point_cloud, temp, pairTransform);
//Convert the current point cloud temp after the registration to the global coordinate system and return result
		pcl::transformPointCloud(*temp, *final_result, GlobalTransform);
//Update the global transformation
		GlobalTransform = GlobalTransform * pairTransform; 
		icp_result = filtered_point_cloud;  //save the last target as the next source
	}
	if(counter % 4 == 0)
	{
	publish_pointcloud(pub_filtered_points_, final_result, input_cloud->header);
	}
	
}


