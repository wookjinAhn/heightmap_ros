//
// Created by wj on 22. 4. 30.
//

#ifndef QUADTREENODE_OLD_H
#define QUADTREENODE_OLD_H

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
#include "HeightmapNode_old.h"

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
		QuadtreeNode(Boundary boundary, int depth, int capacity) : mBoundary(boundary), mDepth(depth), mCapacity(capacity) {}
		QuadtreeNode(Boundary boundary, int depth) : mBoundary(boundary), mDepth(depth) { mCapacity = 1; }
		~QuadtreeNode()
		{
			for (int i = 0; i < mPoints.size(); i++) {
				delete mPoints[i];
			}
			mPoints.clear();

			delete mHeightmap;
			mHeightmap = nullptr;
		}

		std::vector<std::unique_ptr<Point3>> ReadPCDToVector(std::string inputPath);
		std::vector<Point3*> SamplingPoints(std::vector<Point3*> inputPoints, int samplingNum);
		void InsertPoints(std::vector<Point3*> points);
		void WriteHeightmapToPCD(std::string outputPath);

		Boundary GetBoundary() const { return mBoundary; }
		void SetBoundary(Boundary boundary) { mBoundary = boundary; }

		HeightmapNode* GetHeightmap() const { return mHeightmap; }

		std::vector<Point3*> ReadTopicToPoints(sensor_msgs::PointCloud pointcloud_msg);
		void MakeHeightmapToTopic(sensor_msgs::PointCloud& output_pointcloud);

	private:
		void subdivide();
		void insertNode(Point3* point, HeightmapNode* heightmap, int depth);

		HeightmapNode* mHeightmap = new HeightmapNode();
		Boundary mBoundary;
		int mCapacity;
		int mDepth;
		bool mbDivided = false;

		std::vector<Point3*> mPoints;
		std::vector<Point3*> mCapacityPoints;

		std::unique_ptr<QuadtreeNode> mNW = nullptr;
		std::unique_ptr<QuadtreeNode> mNE = nullptr;
		std::unique_ptr<QuadtreeNode> mSW = nullptr;
		std::unique_ptr<QuadtreeNode> mSE = nullptr;
	};
}


#endif //QUADTREENODE_OLD_H
