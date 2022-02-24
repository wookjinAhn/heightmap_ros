#include "../include/heightmap.h"

namespace quadtree
{
    void Heightmap::MakeHeightMap(Point3D* points)	// { <x, z>, <y, dividNum> }
    {
        if (mMapPair.find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ())) == mMapPair.end())	// exist
        {
            int dividNum = 1;
            mMapPair.insert({ std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()), std::make_pair(points->GetY(), dividNum) });
        }
        else	// not exist
        {
            mMapPair.find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()))->second.first += points->GetY();
            mMapPair.find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()))->second.second++;
        }
    }

    void Heightmap::MakeMapToVector()
    {
        for (auto iter = mMapPair.begin(); iter != mMapPair.end(); ++iter)
        {
            Point3D* pointXYZ = new Point3D;
            pointXYZ->SetXYZ(iter->first.first, iter->second.first / iter->second.second, iter->first.second);
            mPoints.push_back(pointXYZ);
        }
    }

    std::vector<std::unique_ptr<Point3D>> Node::ReadPCDToVector(std::string inputPath)
    {
        std::ifstream fin;
        fin.open(inputPath);

        std::vector<std::unique_ptr<Point3D>> inputPoints;
        std::string line;

        if (fin.is_open())
        {
            int num = 1;
            while (!fin.eof())
            {
                getline(fin, line);
                if (num > 10)
                {
                    float x, y, z;
                    std::unique_ptr<Point3D> pointXYZ = std::make_unique<Point3D>();
                    std::istringstream iss(line);
                    iss >> x >> y >> z;
                    pointXYZ->SetXYZ(x, y, z);
                    inputPoints.push_back(std::move(pointXYZ));
                }
                num++;
            }
        }
        fin.close();
        return inputPoints;
    }

    std::vector<Point3D*> Node::SamplingPoints(std::vector<Point3D*> inputPoints, int samplingNum)
    {
        std::vector<Point3D*> samplingPoints;

        std::random_device random;
        std::uniform_int_distribution<int> range(0, inputPoints.size() - 1);

        for (int i = 0; i < samplingNum; i++)
        {
            int randomIndex = range(random);
            samplingPoints.push_back(inputPoints[randomIndex]);
        }
        return samplingPoints;
    }

    void Node::InsertPoints(std::vector<Point3D*> points)
    {
        for (int i = 0; i < points.size(); i++) //
        {
            int depth = 0;
            insertNode(points[i], mHeightmap, depth);	//
        }
        mHeightmap->MakeMapToVector();
    }

    void Node::subdivide()
    {
        float x = mBoundary.GetX();
        float z = mBoundary.GetZ();
        float w = mBoundary.GetW();
        float h = mBoundary.GetH();

        Boundary nw(x - w / 2, z + h / 2, w / 2, h / 2);
        Boundary ne(x + w / 2, z + h / 2, w / 2, h / 2);
        Boundary sw(x - w / 2, z - h / 2, w / 2, h / 2);
        Boundary se(x + w / 2, z - h / 2, w / 2, h / 2);

        mNW = std::make_unique<Node>(nw, mDepth, mCapacity);
        mNE = std::make_unique<Node>(ne, mDepth, mCapacity);
        mSW = std::make_unique<Node>(sw, mDepth, mCapacity);
        mSE = std::make_unique<Node>(se, mDepth, mCapacity);

        mbDivided = true;
    }

    void Node::insertNode(Point3D* point, Heightmap* heightmap, int depth)
    {
        mCapacityPoints.push_back(point);

        if (mDepth == depth)
        {
            heightmap->MakeHeightMap(point);
            return;
        }

        if (mCapacity < mCapacityPoints.size() && mDepth > depth)
        {
            subdivide();
        }

        if (mbDivided)
        {
            while (!mCapacityPoints.empty())
            {
                Point3D* qPoint = mCapacityPoints.back();
                mCapacityPoints.pop_back();
                if (mNW->mBoundary.IsConstained(qPoint))
                {
                    qPoint->SetEndNodeXZ(mNW->GetBoundary().GetX(), mNW->GetBoundary().GetZ());
                    mNW->insertNode(qPoint, heightmap, ++depth);
                }
                else if (mNE->mBoundary.IsConstained(qPoint))
                {
                    qPoint->SetEndNodeXZ(mNE->GetBoundary().GetX(), mNE->GetBoundary().GetZ());
                    mNE->insertNode(qPoint, heightmap, ++depth);
                }
                else if (mSW->mBoundary.IsConstained(qPoint))
                {
                    qPoint->SetEndNodeXZ(mSW->GetBoundary().GetX(), mSW->GetBoundary().GetZ());
                    mSW->insertNode(qPoint, heightmap, ++depth);
                }
                else if (mSE->mBoundary.IsConstained(qPoint))
                {
                    qPoint->SetEndNodeXZ(mSE->GetBoundary().GetX(), mSE->GetBoundary().GetZ());
                    mSE->insertNode(qPoint, heightmap, ++depth);
                }
            }
        }
    }

    void Node::WriteVectorToPCD(std::string outputPath)
    {
        time_t t;
        struct tm* timeinfo;
        time(&t);
        timeinfo = localtime(&t);

        std::string hour, min;

        if (timeinfo->tm_hour < 10) hour = "0" + std::to_string(timeinfo->tm_hour);
        else hour = std::to_string(timeinfo->tm_hour);

        if (timeinfo->tm_min < 10) min = "0" + std::to_string(timeinfo->tm_min);
        else min = std::to_string(timeinfo->tm_min);

        std::string filePath = outputPath + hour + min + "_" + std::to_string(ros::Time::now().nsec / 1000000) + ".pcd";

        std::ofstream fout;
        fout.open(filePath);

        fout << "VERSION" << std::endl;
        fout << "FIELDS x y z" << std::endl;
        fout << "SIZE 4 4 4" << std::endl;
        fout << "TYPE F F F" << std::endl;
        fout << "COUNT 1 1 1" << std::endl;
        fout << "WIDTH 1" << std::endl;
        fout << "HEIGHT " << mHeightmap->GetPoints().size() << std::endl;
        fout << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
        fout << "POINTS " << mHeightmap->GetPoints().size() << std::endl;
        fout << "DATA ascii" << std::endl;

        for (int i = 0; i < mHeightmap->GetPoints().size(); i++)
        {
            fout << mHeightmap->GetPoints()[i]->GetX() << " " << mHeightmap->GetPoints()[i]->GetY() << " " << mHeightmap->GetPoints()[i]->GetZ() << "\n";
        }

        fout.close();
    }

    std::vector<Point3D*> Node::ReadTopicToPoints(sensor_msgs::PointCloud pointcloud_msg)
    {
        for(int i = 0; i < pointcloud_msg.points.size(); i++)
        {
            Point3D* pointXYZ = new Point3D(pointcloud_msg.points[i].x, pointcloud_msg.points[i].y, pointcloud_msg.points[i].z);
            mPoints.push_back(pointXYZ);
        }
        return mPoints;
    }

    void Node::MakeHeightmapToTopic(sensor_msgs::PointCloud& output_pointcloud)
    {
        output_pointcloud.header.frame_id = "camera_depth_optical_frame";           // header
        output_pointcloud.header.stamp = ros::Time::now();
        output_pointcloud.points.resize(mHeightmap->GetPoints().size());     // points

        for (int i = 0; i < output_pointcloud.points.size(); i++)
        {
            output_pointcloud.points[i].x = mHeightmap->GetPoints()[i]->GetX();
            output_pointcloud.points[i].y = mHeightmap->GetPoints()[i]->GetY();
            output_pointcloud.points[i].z = mHeightmap->GetPoints()[i]->GetZ();
        }
        sensor_msgs::ChannelFloat32 outputChannel;  // for custom channels
        outputChannel.name = "distance";
        for (int i = 0; i < output_pointcloud.points.size(); i++)
        {
            outputChannel.values.push_back(output_pointcloud.points[i].x);  // or set to a random value if you like // maybe whatever you want?
        }
        output_pointcloud.channels.push_back(outputChannel);
    }
}
