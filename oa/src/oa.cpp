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

using namespace std;

class gpscalc{

    private:
        float lat_cur;
        float lon_cur;
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

        void set_latcur_loncur(float lat,float lon){
            lat_cur=lat;
            lon_cur=lon;
        }
        void set_latf_lonf(float lat,float lon){
            lat_f=lat;
            lon_f=lon;
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

class lidarcalc{

    private:
    public:
        vector<float> regions;
};

class navigation{
    private:
        ros::Subscriber imu_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber lidar_sub;
        ros::Publisher cmd_vel_pub;
    public:
        navigation(ros::NodeHandle *n){
            imu_sub = n->subscribe("/imu", 100, &navigation::imuCallback,this);
            gps_sub = n->subscribe("/gps/fix", 100, &navigation::gpsCallback,this);
            lidar_sub = n->subscribe("/scan", 100, &navigation::lidarCallback,this);
            cmd_vel_pub = n->advertise<geometry_msgs::Twist>("/cmd_vel", 100);
        }
        gpscalc gps;
        imucalc i;
        lidarcalc l;
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

        void getcoords(){
            float laf,lof;
            cout<<"Enter coordinates: "<<endl;
            cin>>laf;
            cin>>lof;
            gps.set_latf_lonf(laf,lof);
        }

        void att(){
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

            cmd_vel_pub.publish(msg);
            ros::spinOnce();

        }
        void turnleft(){
            geometry_msgs::Twist msg;
            msg.linear.x=0;
            msg.angular.z=0.5;
            cmd_vel_pub.publish(msg);
            ros::spinOnce();        
        }
        void wallfollow(){
            geometry_msgs::Twist msg;
            msg.angular.x=0;
            msg.linear.x=0.5;
            cmd_vel_pub.publish(msg);
            ros::spinOnce();
        }
        
        void setstate(int state){
            if (state==0){
                att();
            }
            else if (state==1){
                turnleft();
            }
            else if (state==2){
                wallfollow();
            }
        }

        void detect(){
            string obstacle;
            float d=1.5;
            if (gps.get_distance(gps.coords.lat_cur,gps.coords.lon_cur) < d){
                setstate(0);
            }
            else{
                if (l.regions.at(1)>d && l.regions.at(2)>d && l.regions.at(3)>d){
                    obstacle="Nothing";
                    setstate(0);
                }
                else if (l.regions.at(1)>d && l.regions.at(2)<d && l.regions.at(3)>d){
                    obstacle="Front";
                    setstate(1);
                }
                else if (l.regions.at(1)<d && l.regions.at(2)>d && l.regions.at(3)>d){
                    obstacle="Front_Right";
                    setstate(2);
                }
                else if (l.regions.at(1)>d && l.regions.at(2)>d && l.regions.at(3)<d){
                    obstacle="Front_Left";
                    setstate(1);
                }
                else if (l.regions.at(1)<d && l.regions.at(2)<d && l.regions.at(3)>d){
                    obstacle="Front and Front_Right";
                    setstate(1);
                }
                else if (l.regions.at(1)>d && l.regions.at(2)<d && l.regions.at(3)<d){
                    obstacle="Front and Front_Left";
                    setstate(1);
                }
                else if (l.regions.at(1)<d && l.regions.at(2)>d && l.regions.at(3)<d){
                    obstacle="Front_Left and Front_Right";
                    setstate(2);
                }
                else if (l.regions.at(1)<d && l.regions.at(2)<d && l.regions.at(3)<d){
                    obstacle="Front_Left and Front and Front_Right";
                    setstate(1);
                }
            }

            
            ROS_INFO("%s",obstacle.c_str());
        }

        void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
            const float max=10;
            l.regions = {
                min(*min_element(msg->ranges.begin(),msg->ranges.begin()+108),max),
                min(*min_element(msg->ranges.begin()+108,msg->ranges.begin()+216),max),
                min(*min_element(msg->ranges.begin()+216,msg->ranges.begin()+324),max),
                min(*min_element(msg->ranges.begin()+324,msg->ranges.begin()+432),max),
                min(*min_element(msg->ranges.begin()+432,msg->ranges.begin()+540),max)          
            };
            detect();
        }

};


int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "nav_node");
    ros::NodeHandle n;
    navigation robot=navigation(&n);
    robot.getcoords();
    ros::spin();
    return 0;
}