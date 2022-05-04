//
// Created by wj on 22. 5. 4.
//

#include "../include/HeightmapNode.h"

namespace camel
{
	HeightmapNode::HeightmapNode(QuadtreeNode* quadtreeNode)
		: mQuadtreeNode(quadtreeNode)
	{
	}

	HeightmapNode::HeightmapNode()
	{
		QuadtreeNode* quadtreeNode = new QuadtreeNode();
		mQuadtreeNode = quadtreeNode;
	}

	HeightmapNode::~HeightmapNode()
	{
		delete mQuadtreeNode;
		mQuadtreeNode = nullptr;

		for (int i = 0; i < mInputPoints.size(); i++) {
			delete mInputPoints[i];
		}
		mInputPoints.clear();
	}

	std::vector<Point3*> HeightmapNode::GetInputPoints() const
	{
		return mInputPoints;
	}

	std::vector<Point3*> HeightmapNode::GetOutputPoints() const
	{
		return mOutputPoints;
	}

	void HeightmapNode::SamplingPoints(std::vector<Point3*>* outputPoints, int samplingNum)
	{
		std::random_device random;
		std::uniform_int_distribution<int> range(0, mInputPoints.size() - 1);

		float degree = -45.0f;
		float rotationRow1[3] = {1.0f, 0.0f, 0.0f};
		float rotationRow2[3] = {0.0f, (float)std::cos(degree * D2R), (float)-std::sin(degree * D2R)};
		float rotationRow3[3] = {0.0f, (float)std::sin(degree * D2R), (float)std::cos(degree * D2R)};

		for (int i = 0; i < samplingNum; i++)
		{
			int randomIndex = range(random);
			Point3* pointXYZ = new Point3(rotationRow1[0] * mInputPoints[randomIndex]->GetX() + rotationRow1[1] * mInputPoints[randomIndex]->GetY() + rotationRow1[2] * mInputPoints[randomIndex]->GetZ(),
				rotationRow2[0] * mInputPoints[randomIndex]->GetX() + rotationRow2[1] * mInputPoints[randomIndex]->GetY() + rotationRow2[2] * mInputPoints[randomIndex]->GetZ(),
				rotationRow3[0] * mInputPoints[randomIndex]->GetX() + rotationRow3[1] * mInputPoints[randomIndex]->GetY() + rotationRow3[2] * mInputPoints[randomIndex]->GetZ());
			outputPoints->push_back(pointXYZ);
		}
//		return samplingPoints;
	}

	void HeightmapNode::FromPCD(std::string inputPath)
	{
		std::ifstream fin;
		fin.open(inputPath);

//		std::vector<std::unique_ptr<Point3>> inputPoints;
		mInputPoints.reserve(307200);
		std::string line;

		if (fin.is_open())
		{
			int num = 1;
			while (!fin.eof())
			{
				getline(fin, line);
				if (num > 10)
				{
					float x, y, z;
					std::istringstream iss(line);
					iss >> x >> y >> z;
					auto* pointXYZ = new Point3(x, y, z);
//					pointXYZ->SetXYZ(x, y, z);
					mInputPoints.push_back(pointXYZ);
				}
				num++;
			}
		}
		fin.close();
	}

	void HeightmapNode::ToPCD(std::string outputPath)
	{
		time_t t;
		struct tm* timeinfo;
		time(&t);
		timeinfo = localtime(&t);

		std::string hour, min;

		if (timeinfo->tm_hour < 10) hour = "0" + std::to_string(timeinfo->tm_hour);
		else hour = std::to_string(timeinfo->tm_hour);

		if (timeinfo->tm_min < 10) min = "0" + std::to_string(timeinfo->tm_min);
		else min = std::to_string(timeinfo->tm_min);

		std::string filePath = outputPath + hour + min + "_" + std::to_string(ros::Time::now().nsec / 1000000) + ".pcd";

		std::ofstream fout;
		fout.open(filePath);

		fout << "VERSION" << std::endl;
		fout << "FIELDS x y z" << std::endl;
		fout << "SIZE 4 4 4" << std::endl;
		fout << "TYPE F F F" << std::endl;
		fout << "COUNT 1 1 1" << std::endl;
		fout << "WIDTH 1" << std::endl;
		fout << "HEIGHT " << mOutputPoints.size() << std::endl;
		fout << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
		fout << "POINTS " << mOutputPoints.size() << std::endl;
		fout << "DATA ascii" << std::endl;

		for (int i = 0; i < mOutputPoints.size(); i++)
		{
			fout << mOutputPoints[i]->GetX() << " " << mOutputPoints[i]->GetY() << " " << mOutputPoints[i]->GetZ() << "\n";
		}

		fout.close();
	}

	void HeightmapNode::FromTopic(sensor_msgs::PointCloud inputPointcloud)
	{
		for(int i = 0; i < inputPointcloud.points.size(); i++)
		{
			Point3* pointXYZ = new Point3(inputPointcloud.points[i].x, inputPointcloud.points[i].y, inputPointcloud.points[i].z);
			mInputPoints.push_back(pointXYZ);
		}
//		return mPoints;
	}

	void HeightmapNode::ToTopic(sensor_msgs::PointCloud& outputPointcloud)
	{
		outputPointcloud.header.frame_id = "camera_depth_optical_frame";           // header
		outputPointcloud.header.stamp = ros::Time::now();
		outputPointcloud.points.resize(mOutputPoints.size());     // points

		for (int i = 0; i < outputPointcloud.points.size(); i++)
		{
			outputPointcloud.points[i].x = mOutputPoints[i]->GetX();
			outputPointcloud.points[i].y = mOutputPoints[i]->GetY();
			outputPointcloud.points[i].z = mOutputPoints[i]->GetZ();
		}
		sensor_msgs::ChannelFloat32 outputChannel;  // for custom channels
		outputChannel.name = "heightmap";
		for (int i = 0; i < outputPointcloud.points.size(); i++)
		{
			outputChannel.values.push_back(outputPointcloud.points[i].x);  // or set to a random value if you like // maybe whatever you want?
		}
		outputPointcloud.channels.push_back(outputChannel);
	}

	void HeightmapNode::insertQuadtreeNode(std::vector<Point3*> points)
	{
		for(int i = 0; i < points.size(); i++)
		{
			int depth = 0;
			mQuadtreeNode->insertNode(points[i], depth, &mMapData);
		}
		makeMapToVector();
	}

	void HeightmapNode::makeMapToVector()
	{
		for (auto iter = mMapData.begin(); iter != mMapData.end(); ++iter)
		{
			Point3* pointXYZ = new Point3(iter->first.first, iter->second, iter->first.second);
			mOutputPoints.push_back(pointXYZ);
		}
	}
}