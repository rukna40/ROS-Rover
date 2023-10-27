#include <iostream>
#include <cmath>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip> 


using namespace std;


class pclcalc{
    private:
        
    
    public:
       

        sensor_msgs::PointCloud2 pclProcessing(const sensor_msgs::PointCloud2ConstPtr& input){

            pcl::PCLPointCloud2Ptr cloud (new pcl::PCLPointCloud2());
            pcl::PCLPointCloud2Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>), cloud_pr (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

            // Convert to PCL data type
            pcl_conversions::toPCL(*input, *cloud);
            
            // Perform the actual filtering
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud (cloud);
            sor.setLeafSize (0.1, 0.1, 0.1);
            sor.filter (*cloud_filtered);

            pcl::fromPCLPointCloud2 (*cloud_filtered,*cloud_xyz);

            pcl::ModelCoefficients::Ptr coeff_p(new pcl::ModelCoefficients ());
            pcl::PointIndices::Ptr inliers_p (new pcl::PointIndices ());

            pcl::SACSegmentation<pcl::PointXYZ> seg_pln;

            seg_pln.setOptimizeCoefficients (true);

            seg_pln.setModelType (pcl::SACMODEL_PLANE);
            seg_pln.setMethodType (pcl::SAC_RANSAC);
            seg_pln.setMaxIterations (1000);
            seg_pln.setDistanceThreshold (0.01);         

            seg_pln.setInputCloud (cloud_xyz);
            seg_pln.segment (*inliers_p, *coeff_p);

            pcl::ExtractIndices<pcl::PointXYZ> extract_p;

            // Extract the inliers
            extract_p.setInputCloud (cloud_xyz);
            extract_p.setIndices (inliers_p);
            extract_p.setNegative (true);
            extract_p.filter (*cloud_pr);

            sensor_msgs::PointCloud2 output_msg;
            output_msg.header = input->header;

            pcl::toROSMsg(*cloud_pr, output_msg);
            
            // pcl::toROSMsg(*cloud_pr, output_msg);

            // pcl_conversions::fromPCL(*cloud_filtered, output_msg);

            return (output_msg);
        }
 
};

class navigation{
    private:

        ros::Subscriber kinect_sub;
        ros::Publisher kinect_pub;

    public:
        navigation(ros::NodeHandle *n){
            kinect_sub = n->subscribe("/kinect/depth/points", 100, &navigation::kinectCallback,this);
            kinect_pub = n->advertise<sensor_msgs::PointCloud2> ("/output", 100);
        }
        pclcalc p;

        void kinectCallback(const sensor_msgs::PointCloud2ConstPtr& input){
            sensor_msgs::PointCloud2 output;
            output = p.pclProcessing(input);
            kinect_pub.publish(output);
        }
        
};

int main(int argc, char **argv){ 

    ros::init(argc, argv, "nav_node");
    ros::NodeHandle n;
    navigation robot=navigation(&n);   
    
    ros::spin();
    return 0;
}