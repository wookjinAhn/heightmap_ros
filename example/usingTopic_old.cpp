//
// Created by wj on 22. 5. 4.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include "../include/heightmap_old.h"
#include "../include/QuadtreeNode_old.h"

ros::Publisher pub;

void pc_callback (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
{
	std::cout << ros::Time::now() << std::endl;
    float MIN_X = -0.5f;
    float MAX_X = 0.5f;
    float Z = 1.0f;
    int DEPTH = 6;
    camel::Boundary boundary(MIN_X, MAX_X, Z);
	camel::QuadtreeNode_old qt(boundary, DEPTH);

    sensor_msgs::PointCloud pointcloud_msg;
    sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud2_msg, pointcloud_msg);  // convert for use XYZ

    std::vector<camel::Point3*> inputQPoints = qt.ReadTopicToPoints(pointcloud_msg);
    std::vector<camel::Point3*> samplingPoints = qt.SamplingPoints(inputQPoints, 5000);
    qt.InsertPoints(samplingPoints);

    sensor_msgs::PointCloud output_pointcloud;
    sensor_msgs::PointCloud2 output_pointcloud2;

    qt.MakeHeightmapToTopic(output_pointcloud);     // set PointCloud msgs for publisher

    sensor_msgs::convertPointCloudToPointCloud2(output_pointcloud, output_pointcloud2);

    pub.publish(output_pointcloud2);

//    // save .pcd
//    std::string outputPath = "/home/wj/Desktop/Data/output_data/";
//    qt.WriteHeightmapToPCD(outputPath);

    std::cout << ros::Time::now() << std::endl;     // for check publish well
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pub_test");

	auto nh = ros::NodeHandle();
	pub = nh.advertise<sensor_msgs::PointCloud2>("heightmap2topic", 10);

	ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, pc_callback);

	ros::spin();
}