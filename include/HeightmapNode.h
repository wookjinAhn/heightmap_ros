//
// Created by wj on 22. 4. 30.
//

#ifndef HEIGHTMAPNODE_H
#define HEIGHTMAPNODE_H

#include <vector>
#include <map>
#include <fstream>
#include "Point3.h"

class HeightmapNode
{
public:
//        ~HeightmapNode();

	std::vector<Point3D*> GetPoints() const;
	void SetPoints(Point3D* point);

	void MakeHeightMap(Point3D* points);
	void MakeMapToVector();

	void WriteVectorToPCD(const std::string &outputPath);

private:
	std::map<std::pair<float, float>, float> mMapPair;
	std::vector<Point3D*> mPoints;
};

#endif //HEIGHTMAPNODE_H
