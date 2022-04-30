//
// Created by wj on 22. 4. 30.
//

#ifndef POINT2_H
#define POINT2_H

#include <camel-euclid/Vector.h>


namespace camel
{
	class Point2 : public CamelVector::Point2D
	{
	public:
		Point2();
		Point2(float x, float z);
	};
}

#endif //POINT2_H
