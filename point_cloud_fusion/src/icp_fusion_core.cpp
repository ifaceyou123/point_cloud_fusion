#include "icp_fusion_core.h"

PointCloudFusion::PointCloudFusion(ros::NodeHandle &nh):source_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),counter(0),GlobalTransform(Eigen::Matrix4f::Identity()){
	
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

void PointCloudFusion::point_cloud_fusion(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_out, Eigen::Matrix4f &final_transform)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZRGB>);

//Crop the cloud_src point cloud image
        std::vector<int> index;

        cloud_src->width = 640;
        cloud_src->height = 480;

        std::cout << "PointCloud src before filtering: " << cloud_src->points.size() << " data points." << std::endl;
        std::cout << "PointCloud tgt before filtering: " << cloud_tgt->points.size() << " data points." << std::endl;

        for( size_t i = 120; i < 360; ++i)
           {
	     for( size_t j = 160; j < 480; ++j)
	        {
	         index.push_back(i*640+j);
                }
           }

        boost::shared_ptr<std::vector<int> > index_ptr = boost::make_shared<std::vector<int> >(index);
//Create a split object
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_src);
        extract.setIndices (index_ptr);
// The indices_rem array indexes all points of cloud_in that are indexed by indices_in
        extract.setNegative (false);
        extract.filter (*cloud_p);

//Remove invalid points from NaNs
  	std::vector<int> indices;
 	pcl::removeNaNFromPointCloud(*cloud_p,*cloud_p, indices);

//use voxel to filter the pointcloud, which from the sensor_msgs::pointcloud2
	pcl::VoxelGrid<pcl::PointXYZRGB> grid;

        grid.setLeafSize (0.07f, 0.07f, 0.07f);
        grid.setInputCloud (cloud_p);
        grid.filter (*src);

        grid.setLeafSize (0.01f, 0.01f, 0.01f);
        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
	//tgt = cloud_tgt;

  	std::cout << "PointCloud src after filtering: " << src->points.size() << " data points." << std::endl;
  	std::cout << "PointCloud tgt after filtering: " << tgt->points.size() << " data points." << std::endl;




//use ICP algorithm to fusion the point cloud 
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//Set the convergence judgment condition. The smaller the smaller the precision, the slower the convergence.
	icp.setTransformationEpsilon (1e-6);
//Set the maximum distance between two correspondences (src<->tgt) to 20cm
	icp.setMaxCorrespondenceDistance (0.2);
//set the maximum number of iterations
	icp.setMaximumIterations (50);  
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	icp.setInputSource (src);   
	icp.setInputTarget (tgt); 

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
	icp.align (*result);

//accumulate transformation between each Iteration
	Ti = icp.getFinalTransformation () * Ti;

// Get the transformation from target to source
	targetToSource = Ti.inverse();

// Transform target back in source frame
	pcl::transformPointCloud (*cloud_tgt, *temp_out, targetToSource);

	//*temp_out += *cloud_src;

	final_transform = targetToSource;

	std::cout <<  " score: " << icp.getFitnessScore() << std::endl;
}

// call back the point cloud and filter it, then use ICP to fusion point cloud to generate map 
void PointCloudFusion::point_cb(const sensor_msgs::PointCloud2ConstPtr & input_cloud){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_result(new pcl::PointCloud<pcl::PointXYZRGB>);

	Eigen::Matrix4f pairTransform;
// transform the pointcloud from ros type to pcl type
	pcl::fromROSMsg(*input_cloud, *target_point_cloud);

	
	counter = counter + 1 ;

	if(counter == 1)
	{
//save the first frame as the source frame
		source_point_cloud = target_point_cloud; 
		cout<<"First Frame has been processed"<<endl;
	}
	else
	{
		point_cloud_fusion(source_point_cloud, target_point_cloud, temp, pairTransform);
//Convert the current point cloud temp after the registration to the global coordinate system and return the result
		pcl::transformPointCloud(*temp, *final_result, GlobalTransform);
//Update global transformation
		GlobalTransform = GlobalTransform * pairTransform; 
		std::cout << "The final GlobalTransform is :" << GlobalTransform << std::endl;
//save the last target as next source
		source_point_cloud = target_point_cloud;  
	}

	publish_pointcloud(pub_filtered_points_, final_result, input_cloud->header);

	if(counter == 30)
	{
		std::cout << "Complete the fusion, the number of frames is 30, program is exited" << std::endl;
		exit(1);
	}
	
}


