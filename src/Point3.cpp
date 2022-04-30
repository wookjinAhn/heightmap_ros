//
// Created by wj on 22. 4. 30.
//

#include "../include/Point3.h"

Point3D::Point3D()
		: CamelVector::Vector3D()
{
}

Point3D::Point3D()
		: CamelVector::Vector3D(x, y, z)
{
}

Point2D Point3D::GetEndNodeXZ() const
{
	return mEndNodeXY;
}

void Point3D::SetEndNodeXY(const Point2D endNodeXY)
{
	mEndNodeXY = endNodeXY;
}

void Point3D::SetEndNodeXY(const float x, const float y)
{
	mEndNodeXY.SetX(x);
	mEndNodeXY.SetY(y);
}