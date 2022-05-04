//
// Created by wj on 22. 5. 4.
//

#ifndef HEIGHTMAPNODE_H
#define HEIGHTMAPNODE_H

#include <vector>
#include <map>
#include <fstream>
#include "Point3.h"
#include "QuadtreeNode.h"

namespace camel
{
	class HeightmapNode
	{
	public:
		HeightmapNode(QuadtreeNode* quadtreeNode);
		HeightmapNode();
		~HeightmapNode();

		std::vector<Point3*> GetInputPoints() const;
		std::vector<Point3*> GetOutputPoints() const;
		std::map<std::pair<float, float>, float>* GetMapData() { return &mMapData; }
		QuadtreeNode* GetQuadtreeNode() { return mQuadtreeNode; }

		void SamplingPoints(std::vector<Point3*>* outputPoints, int samplingNum);	// + rotation

		void FromPCD(std::string inputPath);
		void ToPCD(std::string outputPath);
		void FromTopic(sensor_msgs::PointCloud pointcloud_msg);
		void ToTopic(sensor_msgs::PointCloud& pointcloud_msg);

		void insertQuadtreeNode(std::vector<Point3*> points);
//		void MakeHeightMap(Point3* points);			// == at QuadtreeNode ==

	private:
		void makeMapToVector();

		QuadtreeNode* mQuadtreeNode = nullptr;
		std::map<std::pair<float, float>, float> mMapData;
		std::vector<Point3*> mInputPoints;
		std::vector<Point3*> mOutputPoints;
	};
}

#endif //HEIGHTMAPNODE_H