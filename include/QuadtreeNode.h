//
// Created by wj on 22. 5. 4.
//

#ifndef QUADTREENODE_H
#define QUADTREENODE_H

#include <algorithm>
#include <ctime>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "Point2.h"
#include "Point3.h"
#include "Boundary.h"
//#include "HeightmapNode.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

const double PI = 3.14159265359;
const double D2R = PI / 180;
const double R2D = 180 / PI;

namespace camel
{
	class QuadtreeNode
	{
	public:
		QuadtreeNode(Boundary boundary, int depth, int capacity);
		QuadtreeNode(Boundary boundary, int depth);
		QuadtreeNode(Boundary boundary);
		QuadtreeNode(int depth);
		QuadtreeNode();
//		~QuadtreeNode();

//		std::vector<std::unique_ptr<Point3>> ReadPCDToVector(std::string inputPath);
//		std::vector<Point3*> SamplingPoints(std::vector<Point3*> inputPoints, int samplingNum);
//		void InsertPoints(std::vector<Point3*> points);
//		void WriteHeightmapToPCD(std::string outputPath);

		Boundary GetBoundary() const;
		void SetBoundary(Boundary boundary);
		int GetDepth() const { return mDepth; }

//		HeightmapNode* GetHeightmap() const { return mHeightmap; }

//		std::vector<Point3*> ReadTopicToPoints(sensor_msgs::PointCloud pointcloud_msg);
//		void MakeHeightmapToTopic(sensor_msgs::PointCloud& output_pointcloud);
		void insertNode(Point3* point, int depth, std::map<std::pair<float, float>, float>* mapData);

	private:
		void subdivide();


		void makeHeightmap(Point3* points, std::map<std::pair<float, float>, float>* mapData);

//		HeightmapNode* mHeightmap = new HeightmapNode();
		Boundary mBoundary;
		int mCapacity;
		int mDepth;
		bool mDivided = false;

		std::vector<Point3*> mPoints;
		std::vector<Point3*> mCapacityPoints;

		std::unique_ptr<QuadtreeNode> mNW = nullptr;
		std::unique_ptr<QuadtreeNode> mNE = nullptr;
		std::unique_ptr<QuadtreeNode> mSW = nullptr;
		std::unique_ptr<QuadtreeNode> mSE = nullptr;
	};
}


#endif //QUADTREENODE_H
