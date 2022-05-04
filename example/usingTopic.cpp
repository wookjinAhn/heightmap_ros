#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "../include/heightmap.h"

ros::Publisher pub;

void pc_callback (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
{
	camel::HeightmapNode heightmap;

	sensor_msgs::PointCloud pointcloud_msg;
    sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud2_msg, pointcloud_msg);  // convert for use XYZ

	heightmap.FromTopic(pointcloud_msg);
	std::vector<camel::Point3*> samplingPoints;
	heightmap.SamplingPoints(&samplingPoints, 5000);
	heightmap.insertQuadtreeNode(samplingPoints);

    sensor_msgs::PointCloud output_pointcloud;
    sensor_msgs::PointCloud2 output_pointcloud2;

	heightmap.ToTopic(output_pointcloud);
	sensor_msgs::convertPointCloudToPointCloud2(output_pointcloud, output_pointcloud2);
	pub.publish(output_pointcloud2);
	std::cout << ros::Time::now() << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_test");

    auto nh = ros::NodeHandle();
    pub = nh.advertise<sensor_msgs::PointCloud2>("heightmap2topic", 10);

    ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, pc_callback);

    ros::spin();
}