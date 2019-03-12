#include "icp_fusion_core.h"

PointCloudFusion::PointCloudFusion(ros::NodeHandle &nh):source_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),counter(0),GlobalTransform(Eigen::Matrix4f::Identity()),
transform(tf::StampedTransform()){
	
//subscribe sensor_msgs::pointcloud2
	sub_point_cloud_ = nh.subscribe("/iris_1/camera/depth/points", 100, &PointCloudFusion::point_cb, this);

//publish the filtered point cloud to ros
	pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 50);

	ros::spin();
}

PointCloudFusion::~PointCloudFusion(){}

void PointCloudFusion::Spin(){

}

//publish different point cloud topic, const std_msgs::Header &in_header
void PointCloudFusion::publish_pointcloud(const ros::Publisher &in_publisher, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_to_publish , const std_msgs::Header &in_header)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*input_cloud_to_publish, cloud_msg);
	cloud_msg.header = in_header;
	cloud_msg.header.frame_id = "camera0";
	in_publisher.publish(cloud_msg);
}

void PointCloudFusion::point_cloud_fusion(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_out, Eigen::Matrix4f &final_transform)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_tgt(new pcl::PointCloud<pcl::PointXYZRGB>);




//Crop the cloud_src point cloud image
        std::vector<int> index;

        cloud_src->width = 640;
        cloud_src->height = 480;

        std::cout << "PointCloud src before filtering: " << cloud_src->points.size() << " data points." << std::endl;
        std::cout << "PointCloud tgt before filtering: " << cloud_tgt->points.size() << " data points." << std::endl;

        for( size_t i = 120; i < 360; ++i)
           {
	     for( size_t j = 80; j < 560; ++j)
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

        grid.setLeafSize (0.1f, 0.1f, 0.1f);
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
//Set the convergence judgment condition. The smaller the precision, the slower the convergence.
//it places a limit on the change in fitness score rather than the absolute score.
	icp.setTransformationEpsilon (1e-10);
//Set the maximum distance between two correspondences (src<->tgt) to 20cm
	icp.setEuclideanFitnessEpsilon(0.00001);
	icp.setMaxCorrespondenceDistance (0.2);
//set the maximum number of iterations
	icp.setMaximumIterations (100);  
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
	grid.setLeafSize (0.05f, 0.05f, 0.05f);
        grid.setInputCloud (cloud_tgt);
        grid.filter (*new_tgt);
	pcl::transformPointCloud (*new_tgt, *temp_out, targetToSource);

	//*temp_out += *cloud_src;

	final_transform = targetToSource;
//Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
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

		ros::Time start_time = ros::Time::now();
		std::cout<< "start_time: "<< start_time << std::endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_filtered_point(new pcl::PointCloud<pcl::PointXYZRGB>);
		source_point_cloud = target_point_cloud; 
		cout<<"First Frame has been processed"<<endl;
//		pcl::VoxelGrid<pcl::PointXYZRGB> grid;

//        	grid.setLeafSize (0.05f, 0.05f, 0.05f);
//        	grid.setInputCloud (source_point_cloud);
//        	grid.filter (*source_filtered_point);
//		publish_pointcloud(pub_filtered_points_, source_filtered_point, input_cloud->header);
//		sensor_msgs::PointCloud2 first_frame_cloud;
//		pcl::toROSMsg(*source_filtered_point, first_frame_cloud);

//		tf::StampedTransform transform;
		

        	try {
// Transform msg from camera frame to world frame
   			ros::Time now = ros::Time::now();
   			listener_.waitForTransform("map","iris_1/camera_link",  now, ros::Duration(5.0));

//listener.lookupTransform(destination_frame, original_frame, ros::Time(0), transform)
//save the Stampedtransform of tf from destination frame to original frame.
    			listener_.lookupTransform("map","iris_1/camera_link",  now, transform);

			


//    			sensor_msgs::PointCloud2 transformed_msg;

//Transform a sensor_msgs::PointCloud2 dataset from its frame to a given TF target frame.
//    			pcl_ros::transformPointCloud("map", transform, first_frame_cloud, transformed_msg);

//    			pcl::fromROSMsg(transformed_msg, *source_filtered_point);

			//publish_pointcloud(pub_filtered_points_, source_filtered_point, input_cloud->header);
			
		}

		catch (tf::TransformException const& ex) {
    			ROS_ERROR("%s", ex.what());
   			ROS_WARN("Transformation not available (/map to /camera_link)");
 		}
	}
	//else{
//// Publish coordinate transformation ros::Time::now() is the timestamp carried by the conversion. Pass the name of the parent frame as camera0. The child frame is map.
	tf::Quaternion q;
	q.setRPY(-1.58, 0, -1.58);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map","camera0"));

	if(counter % 10 == 0)
	{	
		point_cloud_fusion(source_point_cloud, target_point_cloud, temp, pairTransform);
//Convert the current point cloud temp after the registration to the global coordinate system and return the result
		pcl::transformPointCloud(*temp, *final_result, GlobalTransform);
//Update global transformation
		GlobalTransform = GlobalTransform * pairTransform; 
		std::cout << "The final GlobalTransform is :" << GlobalTransform << std::endl;
//save the last target as next source
		source_point_cloud = target_point_cloud;  

		publish_pointcloud(pub_filtered_points_, final_result, input_cloud->header);

	}
	//}
	if(counter == 100)
	{
		ros::Time running_time = ros::Time::now();
		//double average_time = 0.1*(running_time - start_time) ;
		std::cout<< "Ending_time: "<< running_time <<std::endl;
		std::cout << "Complete the fusion, the number of frames is 10, program is exited" << std::endl;
		//sleep;
		exit(1);
	}
	
}


