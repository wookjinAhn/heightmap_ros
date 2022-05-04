//
// Created by wj on 22. 4. 30.
//

#ifndef HEIGHTMAPNODE_OLD_H
#define HEIGHTMAPNODE_OLD_H

#include <vector>
#include <map>
#include <fstream>
#include "Point3.h"

namespace camel
{
	class HeightmapNode
	{
	public:
		~HeightmapNode()
		{
			for (int i = 0; i < mPoints.size(); i++) {
				delete mPoints[i];
			}
			mPoints.clear();
		}
		std::vector<Point3*> GetPoints() const { return mPoints; }

		void MakeHeightMap(Point3* points);
		void MakeMapToVector();

	private:
		std::map<std::pair<float, float>, std::pair<float, int>> mMapPair;
		std::map<std::pair<float, float>, float> mHeightPair;
		std::vector<Point3*> mPoints;
	};
}


#endif //HEIGHTMAPNODE_OLD_H
