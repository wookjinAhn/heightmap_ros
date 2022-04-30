//
// Created by wj on 22. 4. 30.
//

#include "../include/HeightmapNode.h"

namespace camel
{
	void HeightmapNode::MakeHeightMap(Point3* points)	// { <x, z>, y}
	{
		if (mHeightPair.find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ())) == mHeightPair.end())	// exist
		{
			mHeightPair.insert({ std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()), points->GetY() });
		}
		else	// not exist
		{
			float beforeHeight = mHeightPair.find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()))->second;
			if (beforeHeight > points->GetY())
			{
				mHeightPair.find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()))->second = points->GetY();
			}
		}
	}

//    void Heightmap::MakeMapToVector()
//    {
//        for (auto iter = mMapPair.begin(); iter != mMapPair.end(); ++iter)
//        {
//            Point3D* pointXYZ = new Point3D;
//            pointXYZ->SetXYZ(iter->first.first, iter->second.first / iter->second.second, iter->first.second);
//            mPoints.push_back(pointXYZ);
//        }
//    }

	void HeightmapNode::MakeMapToVector()
	{
		for (auto iter = mHeightPair.begin(); iter != mHeightPair.end(); ++iter)
		{
			Point3* pointXYZ = new Point3;
			pointXYZ->SetXYZ(iter->first.first, iter->second, iter->first.second);
			mPoints.push_back(pointXYZ);
		}
	}

}