//
// Created by wj on 22. 4. 30.
//

#ifndef POINT3_H
#define POINT3_H

#include <camel-euclid/Vector.h>
#include "Point2.h"

namespace camel
{
	class Point3 : public CamelVector::Point3D
	{
	public:
		Point3();
		Point3(float x, float y, float z);

		Point2 GetEndNodeXZ() const;

		void SetEndNodeXZ(Point2 const endNodeXZ);
		void SetEndNodeXZ(float const x, float const y);

	private:
		Point2 mEndNodeXY;
	};
}

#endif //POINT3_H
