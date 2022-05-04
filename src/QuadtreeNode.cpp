//
// Created by wj on 22. 5. 4.
//

#include "../include/QuadtreeNode.h"

namespace camel
{

	QuadtreeNode::QuadtreeNode(Boundary boundary, int depth, int capacity)
		: mBoundary(boundary)
		, mDepth(depth)
		, mCapacity(capacity)
	{
	}

	QuadtreeNode::QuadtreeNode(Boundary boundary, int depth)
		: mBoundary(boundary)
		, mDepth(depth)
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
		Boundary boundary(-1.0f, 1.0f, 2.0f);
		mBoundary = boundary;
		mCapacity = 1;
		mCapacityPoints.reserve(mCapacity);
	}

	QuadtreeNode::QuadtreeNode()
	{
		Boundary boundary(-1.0f, 1.0f, 2.0f);
		mBoundary = boundary;
		mDepth = 6;
		mCapacity = 1;
		mCapacityPoints.reserve(mCapacity);
	}

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

	void QuadtreeNode::insertNode(Point3* point, int depth, std::map<std::pair<float, float>, float>* mapData)
	{
		mCapacityPoints.push_back(point);

		if (mDepth == depth)
		{
//			std::cout << "insertNode mMapData : " << mapData << std::endl;
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

	void QuadtreeNode::makeHeightmap(Point3* points, std::map<std::pair<float, float>, float>* mapData)
	{
		if (mapData->find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ())) == mapData->end())	// exist
		{
			mapData->insert({ std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()), points->GetY() });
		}
		else	// not exist
		{
			float beforeHeight = mapData->find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()))->second;
			if (beforeHeight > points->GetY())
			{
				mapData->find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()))->second = points->GetY();
			}
		}
	}
}