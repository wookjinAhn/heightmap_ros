//
// Created by wj on 22. 4. 30.
//

#include "../include/Point3.h"

namespace camel
{
	Point3::Point3()
		: CamelVector::Point3D()
	{
	}

	Point3::Point3(float x, float y, float z)
		: CamelVector::Point3D(x, y, z)
	{
	}

	Point2 Point3::GetEndNodeXZ() const
	{
		return mEndNodeXY;
	}

	void Point3::SetEndNodeXZ(const Point2 endNodeXZ)
	{
		mEndNodeXY = endNodeXZ;
	}

	void Point3::SetEndNodeXZ(const float x, const float z)
	{
		mEndNodeXY.SetX(x);
		mEndNodeXY.SetZ(z);
	}
}
