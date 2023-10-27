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

class gpscalc{

    private:
        struct coords_init{
            float lat_i; 
            float lon_i;
        };
        struct coords_final{
            float lat_f; 
            float lon_f;
        };
        float lat_f; 
        float lon_f;
        float d;
        float brng;
        float R = 6371000;
        float phi1;
        float phi2;
        float dphi;
        float dlamda;
        struct gps_coords{
            float lat_cur; 
            float lon_cur;
        };   

    public:
        gps_coords coords;
        coords_init c;
        void set_latcur_loncur(float lat,float lon){
            coords.lat_cur=lat;
            coords.lon_cur=lon;
        }
        void set_latf_lonf(float lat,float lon){
            lat_f=lat;
            lon_f=lon;
        }
        void set_lati_loni(float lat,float lon){
            c.lat_i=lat;
            c.lon_i=lon;
        }
        
        float get_latf(){
            return lat_f;
        }
        float get_lonf(){
            return lon_f;
        }
        float get_distance(float lat_cur, float lon_cur){
            phi1 = lat_cur * M_PI/180;
            phi2 = lat_f * M_PI/180;
            dphi = (lat_f-lat_cur) * M_PI/180;
            dlamda = (lon_f-lon_cur) * M_PI/180;
            float a = sin(dphi/2) * sin(dphi/2) + cos(phi1) * cos(phi2) *sin(dlamda/2) * sin(dlamda/2);
            float c = 2 * atan2(sqrt(a), sqrt(1-a));
            d = R * c;
            return d;            
        }
        float get_angle(float lat_cur, float lon_cur){
            phi1 = lat_cur * M_PI/180;
            phi2 = lat_f * M_PI/180;
            dphi = (lat_f-lat_cur) * M_PI/180;
            dlamda = (lon_f-lon_cur) * M_PI/180;
            float y = sin(dlamda)*cos(phi2);
            float x = cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(dlamda);
            float theta = M_PI_2-atan2(y, x);
            brng = fmod((theta*180/M_PI + 360),360);
            return brng;
        }       
};

class imucalc{

    private:
        struct Quaternion {
            double w, x, y, z;
        };

        struct EulerAngles {
            double roll, pitch, yaw;
        };

        float yaw_c; 
               
    public:
        Quaternion values;
        EulerAngles angle;

        float get_yaw_c(){
            return yaw_c;
        }
        void set_yaw_c(float y){
            yaw_c=y;
        }

        EulerAngles ToEulerAngles(Quaternion q) {
            EulerAngles angles;

            double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
            double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
            angles.roll = std::atan2(sinr_cosp, cosr_cosp);

            double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
            double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
            angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            angles.yaw = std::atan2(siny_cosp, cosy_cosp);

            return angles;
        }
};

class pclcalc{
    private:
        double target;
        vector<float> histogram;
        vector<pcl::PointIndices> cluster_indices;
    
    public:
        void set_target(double t){
            target=t;
        }
        double get_target(){
            return target;
        }
        void set_hist(vector<float> hist){
            histogram=hist;
        }
        vector<float> get_hist(){
            return histogram;
        }
        void set_cluster_indices(vector<pcl::PointIndices> ci){
            cluster_indices=ci;
        }
        vector<pcl::PointIndices> get_cluster_indices(){
            return cluster_indices;
        }

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

            // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            // tree->setInputCloud (cloud_pr);

            // vector<pcl::PointIndices> cluster_indices;

            // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            // ec.setClusterTolerance (0.02); // 2cm
            // ec.setMinClusterSize (100);
            // ec.setMaxClusterSize (25000);
            // ec.setSearchMethod (tree);
            // ec.setInputCloud (cloud_pr);
            // ec.extract (cluster_indices);

            // set_cluster_indices(cluster_indices);

            sensor_msgs::PointCloud2 output_msg;
            output_msg.header = input->header;

            pcl::toROSMsg(*cloud_pr, output_msg);
            
            // pcl::toROSMsg(*cloud_pr, output_msg);

            // pcl_conversions::fromPCL(*cloud_filtered, output_msg);

            return (output_msg);
        }

        void gen_histogram(const sensor_msgs::PointCloud2& input){

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(input, *cloud);

            vector<float> mindistances;
            for (const auto& cluster : cluster_indices)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                float mindis;
                
                for (const auto& idx : cluster.indices) {
                    pcl::PointXYZ point=(*cloud)[idx];

                    float dis=sqrt(point.x*point.x + point.z*point.z);
                    float angle=atan2(point.z,point.x)*(180/M_PI);
                    
                    if (dis<mindis){
                        mindis=dis;
                    }

                    cloud_cluster->push_back(point);   
                }
                mindistances.push_back(mindis);
            }            
            
            // vector<float> histogram(180,0);

            // for(const auto& point: *cloud){
            //     // ROS_INFO("x : %f y : %f z : %f",point.x, point.y, point.z);
            //     float dist,angle;

            //     dist=sqrt(point.x*point.x + point.z*point.z);
            //     angle=atan2(point.z,point.x)*(180/M_PI);
                    
            //     if (angle < 0) {
            //         angle += 360.0;
            //     }
                
                
            //     int sample = (int)angle % 180;
            //     histogram[sample]++;
            // }
            // set_hist(histogram);
            
        }

        double findDirection(vector<float> histogram){
            int minpts=histogram.at(0);
            int index=0;

            for (int i=1;i<180;i++){
                if (histogram.at(i)<minpts){
                    index=i;
                    minpts=histogram.at(i);
                }
            }

            double target=float(index)*(M_PI/180);
            return target;
        }       
};

class navigation{
    private:
        ros::Subscriber imu_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber kinect_sub;
        ros::Publisher kinect_pub;
        ros::Publisher cmd_vel_pub;
    public:
        navigation(ros::NodeHandle *n){
            imu_sub = n->subscribe("/imu", 100, &navigation::imuCallback,this);
            gps_sub = n->subscribe("/gps/fix", 100, &navigation::gpsCallback,this);
            kinect_sub = n->subscribe("/kinect/depth/points", 100, &navigation::kinectCallback,this);
            kinect_pub = n->advertise<sensor_msgs::PointCloud2> ("output", 100);
            cmd_vel_pub = n->advertise<geometry_msgs::Twist>("/cmd_vel", 100);
        }
        gpscalc gps;
        imucalc i;
        pclcalc p;

        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
            i.values.x=msg->orientation.x;
            i.values.y=msg->orientation.y;
            i.values.z=msg->orientation.z;
            i.values.w=msg->orientation.w;
        }

        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
            gps.coords.lat_cur=msg->latitude;
            gps.coords.lon_cur=msg->longitude;
        }

        void kinectCallback(const sensor_msgs::PointCloud2ConstPtr& input){
            sensor_msgs::PointCloud2 output;
            output = p.pclProcessing(input);
            // p.gen_histogram(output);
            // final code add here
            kinect_pub.publish(output);
        }

        void setFinalCoords(){
            float laf,lof;
            cout<<"Enter coordinates: "<<endl;
            cin>>laf;
            cin>>lof;
            gps.set_latf_lonf(laf,lof);
        }
        
        geometry_msgs::Twist att(){
            geometry_msgs::Twist msg;
        
            i.angle=i.ToEulerAngles(i.values);        
            float cur_dis=gps.get_distance(gps.coords.lat_cur,gps.coords.lon_cur);
            float target_rad=gps.get_angle(gps.coords.lat_cur,gps.coords.lon_cur)*M_PI/180;

            if (target_rad>=M_PI){
                target_rad=target_rad-2*M_PI;
            }

            i.set_yaw_c(i.angle.yaw);  
            
            if (abs(cur_dis)<=0.1){
                msg.linear.x=0;
            }
            else if (abs(target_rad-i.get_yaw_c())<=0.1){
                msg.linear.x=0.1*abs(cur_dis);
                msg.angular.z=0;
            }
            else{
                msg.angular.z=2.5*(target_rad-i.get_yaw_c());
            }

            return(msg);
        }

        geometry_msgs::Twist oa(){
            geometry_msgs::Twist msg;

            vector<float> histogram=p.get_hist();
            p.set_target(p.findDirection(histogram));
            double target=p.get_target();

            if (target>=M_PI){
                target=target-2*M_PI;
            }

            if (abs(target-i.get_yaw_c())<=0.1){
                msg.angular.z=0;
                msg.linear.x=1;
            }
            else{
                msg.linear.x=0;
                msg.angular.z=2.5*(target-i.get_yaw_c());
            }

            return(msg);
        }
   
        geometry_msgs::Twist control(){
            geometry_msgs::Twist msg;                                                   //do this now
            if (1){
                msg=att();
            }
            else{
                msg=oa();
            }
            return msg;
        }
};

int main(int argc, char **argv){ 

    ros::init(argc, argv, "nav_node");
    ros::NodeHandle n;
    navigation robot=navigation(&n);
    //robot.setFinalCoords();    
    
    ros::spin();
    return 0;
}