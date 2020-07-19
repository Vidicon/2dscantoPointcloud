#include <ros/ros.h>

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/ChannelFloat32.h"

ros::Publisher g_scan3d_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::LaserScan scan = (sensor_msgs::LaserScan)*msg;
   
    static uint32_t seq = 0;
    static sensor_msgs::PointCloud scan3dMsg;
    scan3dMsg.header.seq = seq++;
    scan3dMsg.header.stamp = ros::Time::now();
    scan3dMsg.header.frame_id = "scan_3d";

    static sensor_msgs::ChannelFloat32 channelIntensity; 
    channelIntensity.name = "intensity";

    //for (float iy = 0; iy < (M_PI/2); iy+=(M_PI/18))
    //{
    //ROS_INFO("angle_min: %f , angle_max %f , angle_increment %f",scan.angle_min,scan.angle_max,scan.angle_increment);
    static float iy = M_PI;

   
    uint16_t pointCount = 0;

    for (float i = scan.angle_min; i < scan.angle_max; i+=scan.angle_increment)
    {
        
        geometry_msgs::Point32 point;

        if(i < (M_PI/2) || i > M_PI + (M_PI/2))
        {
            //ROS_INFO("scan: %f ",scan.ranges[pointCount]);
            if(scan.ranges[pointCount] > 0.005 && scan.ranges[pointCount] < 3)
            {
                float dis = scan.ranges[pointCount];
                point.x = -(sin(i) * dis);//scan.ranges[pointCount];
                point.y = cos(i) * sin(iy) * dis;//
                point.z = cos(iy) * cos(i) * dis;//scan.ranges[pointCount];
                scan3dMsg.points.push_back(point);
                channelIntensity.values.push_back(scan.intensities[pointCount]);
            }
        }
        pointCount++;
        
    }
    
    //}
    iy -= (M_PI/180);
    if(iy < (M_PI/4))
    {
        return;
        iy = M_PI;
    }
    scan3dMsg.channels.clear();
    scan3dMsg.channels.push_back(channelIntensity);
    g_scan3d_pub.publish(scan3dMsg);
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "lidar_transformer");
	ros::NodeHandle n;
	ros::Subscriber scanSubscriber = n.subscribe("scan", 1000, scanCallback);
    
    g_scan3d_pub = n.advertise<sensor_msgs::PointCloud>("scan3d", 1000);

    

    ros::Rate loop_rate(10);
    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
	
	return 0;
}