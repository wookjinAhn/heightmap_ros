//
// Created by wj on 22. 5. 4.
//

#include "../include/QuadtreeNode.h"

namespace camel
{

	QuadtreeNode::QuadtreeNode(Boundary boundary, int depth, int capacity)
		: mBoundary(boundary), mDepth(depth), mCapacity(capacity)
	{
	}

	QuadtreeNode::QuadtreeNode(Boundary boundary, int depth)
		: mBoundary(boundary), mDepth(depth)
	{
		mCapacity = 1;
		mCapacityPoints.reserve(mCapacity);
	}

	QuadtreeNode::QuadtreeNode(Boundary boundary)
		: mBoundary(boundary)
	{
		mDepth = 6;
		mCapacity = 1;
		mCapacityPoints.reserve(mCapacity);
	}

	QuadtreeNode::QuadtreeNode(int depth)
		: mDepth(depth)
	{
		Boundary boundary(2.0f, -1.0f, 1.0f);
		mBoundary = boundary;
		mCapacity = 1;
		mCapacityPoints.reserve(mCapacity);
	}

	QuadtreeNode::QuadtreeNode()
	{
		Boundary boundary(2.0f, -1.0f, 1.0f);
		mBoundary = boundary;
		mDepth = 6;
		mCapacity = 1;
		mCapacityPoints.reserve(mCapacity);
	}

//	QuadtreeNode::~QuadtreeNode()
//	{
//		for (int i = 0; i < mPoints.size(); i++) {
//			delete mPoints[i];
//		}
//		mPoints.clear();
//	}
//
//	std::vector<std::unique_ptr<Point3>> QuadtreeNode::ReadPCDToVector(std::string inputPath)
//	{
//		std::ifstream fin;
//		fin.open(inputPath);
//
//		std::vector<std::unique_ptr<Point3>> inputPoints;
//		std::string line;
//
//		if (fin.is_open())
//		{
//			int num = 1;
//			while (!fin.eof())
//			{
//				getline(fin, line);
//				if (num > 10)
//				{
//					float x, y, z;
//					std::unique_ptr<Point3> pointXYZ = std::make_unique<Point3>();
//					std::istringstream iss(line);
//					iss >> x >> y >> z;
//					pointXYZ->SetXYZ(x, y, z);
//					inputPoints.push_back(std::move(pointXYZ));
//				}
//				num++;
//			}
//		}
//		fin.close();
//		return inputPoints;
//	}

//	std::vector<Point3*> QuadtreeNode::SamplingPoints(std::vector<Point3*> inputPoints, int samplingNum)
//	{
//		std::vector<Point3*> samplingPoints;
//
//		std::random_device random;
//		std::uniform_int_distribution<int> range(0, inputPoints.size() - 1);
//
//		float degree = -45.0f;
//		float rotationRow1[3] = {1.0f, 0.0f, 0.0f};
//		float rotationRow2[3] = {0.0f, (float)std::cos(degree * D2R), (float)-std::sin(degree * D2R)};
//		float rotationRow3[3] = {0.0f, (float)std::sin(degree * D2R), (float)std::cos(degree * D2R)};
//
//		for (int i = 0; i < samplingNum; i++)
//		{
//			int randomIndex = range(random);
//			Point3* pointXYZ = new Point3(rotationRow1[0] * inputPoints[randomIndex]->GetX() + rotationRow1[1] * inputPoints[randomIndex]->GetY() + rotationRow1[2] * inputPoints[randomIndex]->GetZ(),
//				rotationRow2[0] * inputPoints[randomIndex]->GetX() + rotationRow2[1] * inputPoints[randomIndex]->GetY() + rotationRow2[2] * inputPoints[randomIndex]->GetZ(),
//				rotationRow3[0] * inputPoints[randomIndex]->GetX() + rotationRow3[1] * inputPoints[randomIndex]->GetY() + rotationRow3[2] * inputPoints[randomIndex]->GetZ());
//			samplingPoints.push_back(pointXYZ);
//		}
//		return samplingPoints;
//	}

//	void QuadtreeNode::InsertPoints(std::vector<Point3*> points)
//	{
//		for (int i = 0; i < points.size(); i++) //
//		{
//			int depth = 0;
//
//			insertNode(points[i], mHeightmap, depth);	//
//		}
//		mHeightmap->MakeMapToVector();
//	}

	Boundary QuadtreeNode::GetBoundary() const
	{
		return mBoundary;
	}

	void QuadtreeNode::SetBoundary(Boundary boundary)
	{
		mBoundary = boundary;
	}

	void QuadtreeNode::subdivide()
	{
		float x = mBoundary.GetX();
		float z = mBoundary.GetZ();
		float w = mBoundary.GetW();
		float h = mBoundary.GetH();

		Boundary nw(x - w / 2, z + h / 2, w / 2, h / 2);
		Boundary ne(x + w / 2, z + h / 2, w / 2, h / 2);
		Boundary sw(x - w / 2, z - h / 2, w / 2, h / 2);
		Boundary se(x + w / 2, z - h / 2, w / 2, h / 2);

		mNW = std::make_unique<QuadtreeNode>(nw, mDepth, mCapacity);
		mNE = std::make_unique<QuadtreeNode>(ne, mDepth, mCapacity);
		mSW = std::make_unique<QuadtreeNode>(sw, mDepth, mCapacity);
		mSE = std::make_unique<QuadtreeNode>(se, mDepth, mCapacity);

		mDivided = true;
	}

	void QuadtreeNode::insertNode(Point3* point, int depth, std::map<std::pair<float, float>, float> mapData)
	{
		mCapacityPoints.push_back(point);

		if (mDepth == depth)
		{
			makeHeightmap(point, mapData);
			return;
		}

		if (mCapacity < mCapacityPoints.size() && mDepth > depth)
		{
			subdivide();
		}

		if (mDivided)
		{
			while (!mCapacityPoints.empty())
			{
				Point3* qPoint = mCapacityPoints.back();
				mCapacityPoints.pop_back();
				if (mNW->mBoundary.IsConstained(qPoint))
				{
					qPoint->SetEndNodeXZ(mNW->GetBoundary().GetX(), mNW->GetBoundary().GetZ());
					mNW->insertNode(qPoint, ++depth, mapData);
				}
				else if (mNE->mBoundary.IsConstained(qPoint))
				{
					qPoint->SetEndNodeXZ(mNE->GetBoundary().GetX(), mNE->GetBoundary().GetZ());
					mNE->insertNode(qPoint, ++depth, mapData);
				}
				else if (mSW->mBoundary.IsConstained(qPoint))
				{
					qPoint->SetEndNodeXZ(mSW->GetBoundary().GetX(), mSW->GetBoundary().GetZ());
					mSW->insertNode(qPoint, ++depth, mapData);
				}
				else if (mSE->mBoundary.IsConstained(qPoint))
				{
					qPoint->SetEndNodeXZ(mSE->GetBoundary().GetX(), mSE->GetBoundary().GetZ());
					mSE->insertNode(qPoint, ++depth, mapData);
				}
			}
		}
	}

	void QuadtreeNode::makeHeightmap(Point3* points, std::map<std::pair<float, float>, float> mapData)
	{
		if (mapData.find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ())) == mapData.end())	// exist
		{
			mapData.insert({ std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()), points->GetY() });
		}
		else	// not exist
		{
			float beforeHeight = mapData.find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()))->second;
			if (beforeHeight > points->GetY())
			{
				mapData.find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()))->second = points->GetY();
			}
		}
	}
}