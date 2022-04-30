//
// Created by wj on 22. 4. 30.
//

#ifndef BOUNDARY_H
#define BOUNDARY_H

#include <camel-euclid/Vector.h>
#include "Point3.h"

namespace camel
{
	class Boundary
	{
	public:
		Boundary();
		Boundary(float x, float z, float w, float h);
//        Boundary(float minX, float maxX, float y);
		Boundary(float minX, float maxX, float z);

		float GetX() const;
		float GetZ() const;
		float GetW() const;
		float GetH() const;

		void SetBoundary(float const x, float const z, float const w, float const h);

		bool IsConstained(Point3* p) const;

	private:
		float mX, mZ, mW, mH;
	};
};


#endif //BOUNDARY_H
