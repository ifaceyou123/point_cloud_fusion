#include "icp_fusion_core.h"

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "point_cloud_fusion");

    ros::NodeHandle nh;

    PointCloudFusion core(nh);
   
    return 0;
}
